"""Read-only, opt-in physics-step tracing for VPC oracle experiments."""

import hashlib

import numpy as np


def _pose_values(pose):
    if pose is None:
        return None
    if hasattr(pose, "p") and hasattr(pose, "q"):
        return [float(v) for v in np.r_[pose.p, pose.q]]
    return [float(v) for v in np.asarray(pose).reshape(-1)]


class StepTrace:
    def __init__(self, task, stage_names, stride=25):
        if stride < 1:
            raise ValueError("trace stride must be positive")
        self.task = task
        self.stage_names = set(stage_names)
        self.stride = stride
        self.initial_basket_xy = task.breadbasket.get_pose().p[:2].copy()
        self.initial_bread_z = [float(actor.get_pose().p[2]) for actor in task.bread]
        self.bread_names = {actor.get_name() for actor in task.bread}
        self.bread_index_by_entity_id = {
            actor.actor.get_per_scene_id(): index
            for index, actor in enumerate(task.bread)
        }
        self.basket_name = task.breadbasket.get_name()
        self.gripper_names = set(task.robot.gripper_name)
        self.left_link_names = {
            link.get_name() for link in task.robot.left_entity.get_links()
        }
        self.right_link_names = {
            link.get_name() for link in task.robot.right_entity.get_links()
        }
        self.robot_link_names = {
            *self.left_link_names, *self.right_link_names
        }
        self.records = []
        self.current = None
        self.stage_start_basket_xy = None
        self.seen_contacts = set()
        self.seen_drift = set()
        self.seen_lift = set()
        self.plan_start = None

    def begin(self, name, actions):
        if name not in self.stage_names:
            return
        self.stage_start_basket_xy = self.task.breadbasket.get_pose().p[:2].copy()
        self.seen_contacts.clear()
        self.seen_drift.clear()
        self.seen_lift = {
            index for index, actor in enumerate(self.task.bread)
            if actor.get_pose().p[2] >= self.initial_bread_z[index] + 0.02
        }
        self.plan_start = (len(self.task.left_joint_path), len(self.task.right_joint_path))
        self.current = {
            "stage": name,
            "steps": 0,
            "initial_joint_qpos": {
                "left": [float(v) for v in self.task.robot.left_entity.get_qpos()],
                "right": [float(v) for v in self.task.robot.right_entity.get_qpos()],
            },
            "actions": [
                {"arm": str(arm), "commands": [
                    {"kind": action.action,
                     "target_pose": _pose_values(action.target_pose),
                     "target_gripper_pos": action.target_gripper_pos}
                    for action in commands
                ]}
                for arm, commands in actions if arm is not None
            ],
            "events": [],
            "samples": [],
        }
        self.task.simulation_step_observer = self.after_step

    def _state(self):
        basket = self.task.breadbasket.get_pose().p
        return {
            "basket_xyz": [float(v) for v in basket],
            "basket_drift_m": float(np.linalg.norm(basket[:2] - self.initial_basket_xy)),
            "bread_xyz": [
                [float(v) for v in actor.get_pose().p] for actor in self.task.bread
            ],
            "left_ee_xyz": [float(v) for v in self.task.robot.get_left_ee_pose()[:3]],
            "right_ee_xyz": [float(v) for v in self.task.robot.get_right_ee_pose()[:3]],
        }

    def _event(self, kind, **fields):
        self.current["events"].append({
            "step": self.current["steps"], "kind": kind, **fields,
        })

    def _contact_kind(self, names):
        bread = bool(self.bread_names.intersection(names))
        basket = self.basket_name in names
        gripper = bool(self.gripper_names.intersection(names))
        robot = bool(self.robot_link_names.intersection(names))
        if bread and basket:
            return "bread_basket"
        if bread and gripper:
            return "bread_gripper"
        if basket and gripper:
            return "basket_gripper"
        if basket and robot:
            return "basket_robot"
        if bread and robot:
            return "bread_robot"
        if bread and "table" in names:
            return "bread_table"
        if basket and "table" in names:
            return "basket_table"
        return None

    def after_step(self):
        record = self.current
        record["steps"] += 1
        step = record["steps"]
        basket = self.task.breadbasket.get_pose().p
        stage_drift = float(np.linalg.norm(basket[:2] - self.stage_start_basket_xy))
        for threshold in (0.005, 0.01, 0.05, 0.1):
            if threshold not in self.seen_drift and stage_drift >= threshold:
                self.seen_drift.add(threshold)
                self._event("basket_drift_threshold", threshold_m=threshold,
                            stage_drift_m=stage_drift)
        for index, actor in enumerate(self.task.bread):
            if index not in self.seen_lift and actor.get_pose().p[2] >= self.initial_bread_z[index] + 0.02:
                self.seen_lift.add(index)
                self._event("bread_lift_20mm", bread_index=index)
        for contact in self.task.scene.get_contacts():
            if not contact.points:
                continue
            names = tuple(body.entity.name for body in contact.bodies)
            kind = self._contact_kind(names)
            if kind is not None:
                bread_index = next((
                    self.bread_index_by_entity_id.get(body.entity.get_per_scene_id())
                    for body in contact.bodies
                    if body.entity.get_per_scene_id() in self.bread_index_by_entity_id
                ), None)
                key = (kind, bread_index)
                if key in self.seen_contacts:
                    continue
                self.seen_contacts.add(key)
                arm = (
                    "left" if self.left_link_names.intersection(names)
                    else "right" if self.right_link_names.intersection(names)
                    else None
                )
                self._event(kind, entities=list(names), bread_index=bread_index,
                            arm=arm, point_count=len(contact.points))
        if step == 1 or step % self.stride == 0:
            record["samples"].append({"step": step, **self._state()})

    def end(self):
        self.task.simulation_step_observer = None
        if self.current is not None:
            record = self.current
            def path_info(path):
                if not isinstance(path, dict):
                    return {"status": None, "steps": 0, "sha256": None}
                positions = path.get("position")
                if positions is None:
                    return {"status": path.get("status"), "steps": 0, "sha256": None}
                positions = np.asarray(positions)
                return {
                    "status": path.get("status"),
                    "steps": int(positions.shape[0]),
                    "sha256": hashlib.sha256(positions.tobytes()).hexdigest(),
                }

            left_start, right_start = self.plan_start
            record["planned_joint_paths"] = {
                "left": [path_info(path) for path in self.task.left_joint_path[left_start:]],
                "right": [path_info(path) for path in self.task.right_joint_path[right_start:]],
            }
            if record["steps"] and record["samples"][-1]["step"] != record["steps"]:
                record["samples"].append({"step": record["steps"], **self._state()})
            self.records.append(record)
            self.current = None
