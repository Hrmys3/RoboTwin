"""Export camera observations and audit object visibility for VPC.

Example: python -m scripts.vpc_vision_probe --seeds 0 2
"""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from envs.place_bread_basket import place_bread_basket
from scripts.vpc_oracle import ROOT, load_config
from vpc.place_bread_basket import execute
from vpc.scene import OracleSceneProvider


def cameras_by_name(camera_manager):
    cameras = dict(zip(camera_manager.static_camera_name,
                       camera_manager.static_camera_list))
    if camera_manager.collect_wrist_camera:
        cameras["left_camera"] = camera_manager.left_camera
        cameras["right_camera"] = camera_manager.right_camera
    return cameras


def project_world_point(point, intrinsic, extrinsic):
    world = np.r_[np.asarray(point, dtype=float), 1.0]
    camera = np.asarray(extrinsic, dtype=float) @ world
    if camera[2] <= 0:
        return None
    pixel = np.asarray(intrinsic, dtype=float) @ camera[:3]
    return [float(pixel[0] / pixel[2]), float(pixel[1] / pixel[2])]


def object_visibility(actor, actor_id, labels, depth_m, intrinsic, extrinsic):
    mask = labels == actor_id
    rows, cols = np.nonzero(mask)
    height, width = labels.shape
    center_uv = project_world_point(actor.get_pose().p, intrinsic, extrinsic)
    return {
        "actor_id": int(actor_id),
        "pixel_count": int(mask.sum()),
        "bbox_xyxy": [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())]
        if cols.size else None,
        "depth_median_m": float(np.median(depth_m[mask & (depth_m > 0)]))
        if np.any(mask & (depth_m > 0)) else None,
        "world_center_xyz": [float(x) for x in actor.get_pose().p],
        "projected_center_uv": center_uv,
        "center_inside_image": bool(center_uv is not None and
                                    0 <= center_uv[0] < width and
                                    0 <= center_uv[1] < height),
    }


def capture(task, seed, stage, snapshot_dir):
    task._update_render()
    task.cameras.update_picture()
    camera_config = task.cameras.get_config()
    camera_rgb = task.cameras.get_rgb()
    objects = {f"bread_{index}": actor for index, actor in enumerate(task.bread)}
    objects["basket"] = task.breadbasket
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    report = {"seed": seed, "stage": stage,
              "plan_success_so_far": bool(task.plan_success), "cameras": {}}
    for name, camera in cameras_by_name(task.cameras).items():
        if name not in camera_config or name not in camera_rgb:
            continue
        rgb = camera_rgb[name]["rgb"]
        labels = camera.get_picture("Segmentation")[..., 1].astype(np.int32)
        depth_m = -camera.get_picture("Position")[..., 2]
        if labels.shape != rgb.shape[:2] or depth_m.shape != labels.shape:
            raise ValueError(f"{name}: RGB, actor IDs, and depth have different image sizes")
        depth_m = np.where(np.isfinite(depth_m) & (depth_m > 0), depth_m, 0)
        intrinsic = camera_config[name]["intrinsic_cv"]
        extrinsic = camera_config[name]["extrinsic_cv"]
        Image.fromarray(rgb).save(snapshot_dir / f"{name}_rgb.png")
        np.savez_compressed(snapshot_dir / f"{name}_geometry.npz",
                            actor_ids=labels, depth_m=depth_m.astype(np.float32),
                            intrinsic_cv=intrinsic, extrinsic_cv=extrinsic)
        report["cameras"][name] = {
            "image_hw": list(rgb.shape[:2]),
            "rgb_file": f"{name}_rgb.png",
            "geometry_file": f"{name}_geometry.npz",
            "intrinsic_cv": np.asarray(intrinsic).tolist(),
            "extrinsic_cv": np.asarray(extrinsic).tolist(),
            "objects": {
                object_name: object_visibility(
                    actor, actor.actor.get_per_scene_id(), labels, depth_m,
                    intrinsic, extrinsic)
                for object_name, actor in objects.items()
            },
        }
    (snapshot_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


def image_annotations(report, snapshot_dir, output_dir):
    for camera_name, camera in report["cameras"].items():
        objects = []
        for object_name, data in camera["objects"].items():
            bbox = data["bbox_xyxy"]
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            objects.append({
                "category": "basket" if object_name == "basket" else "bread",
                "instance": object_name,
                "bbox_xywh": [x1, y1, x2 - x1 + 1, y2 - y1 + 1],
                "visible_pixels": data["pixel_count"],
            })
        yield {
            "seed": report["seed"], "stage": report["stage"],
            "camera": camera_name, "image_hw": camera["image_hw"],
            "image": str((snapshot_dir / camera["rgb_file"]).relative_to(output_dir)),
            "objects": objects,
        }


def probe(seed, output_dir, capture_stages=()):
    config = load_config()
    task = place_bread_basket()
    reports = []
    try:
        task.setup_demo(seed=seed, now_ep_num=0, **config)
        seed_dir = output_dir / f"seed_{seed}"
        initial_dir = seed_dir / "initial" if capture_stages else seed_dir
        reports.append((capture(task, seed, "initial", initial_dir), initial_dir))
        if capture_stages:
            requested = set(capture_stages)
            provider = OracleSceneProvider(task, 0.0, seed + 1000003)

            def on_stage(stage):
                if stage in requested:
                    snapshot_dir = seed_dir / stage
                    reports.append((capture(task, seed, stage, snapshot_dir), snapshot_dir))

            branch = execute(task, provider, on_stage=on_stage)
            joints_legal, joint_excess_rad = task.planned_joints_legal()
            outcome = {
                "seed": seed, "branch": branch,
                "plan_success": bool(task.plan_success),
                "task_success": bool(task.check_success()),
                "joints_legal": bool(joints_legal),
                "joint_excess_rad": float(joint_excess_rad),
            }
            outcome["success"] = bool(
                outcome["plan_success"] and outcome["task_success"]
                and outcome["joints_legal"])
            (seed_dir / "episode_outcome.json").write_text(
                json.dumps(outcome, indent=2), encoding="utf-8")
        return reports
    finally:
        if hasattr(task, "scene"):
            task.close_env()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "vpc_results" / "vision_probe")
    parser.add_argument("--capture-stages", nargs="+", default=[],
                        help="Also capture named stages after oracle actions complete")
    args = parser.parse_args()
    if any("/" in stage or stage in {".", "..", "initial"} for stage in args.capture_stages):
        parser.error("capture-stages must be action-stage names, excluding initial")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    annotations_path = args.output_dir / "annotations.jsonl"
    annotations = []
    for seed in args.seeds:
        reports = probe(seed, args.output_dir, args.capture_stages)
        for report, snapshot_dir in reports:
            annotations.extend(image_annotations(report, snapshot_dir, args.output_dir))
            print(json.dumps({"seed": seed, "stage": report["stage"],
                              "visible_pixels": {
                camera: {obj: data["pixel_count"] for obj, data in info["objects"].items()}
                for camera, info in report["cameras"].items()
            }}), flush=True)
    annotations_path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in annotations),
        encoding="utf-8")


if __name__ == "__main__":
    main()
