"""Run paired oracle and position-error trials for place_bread_basket.

Example: python -m scripts.vpc_oracle --seeds 0 2 --sigmas-mm 0 5 10
"""

import argparse
import json
from pathlib import Path

import yaml

from envs.place_bread_basket import place_bread_basket
from vpc.place_bread_basket import execute
from vpc.scene import OracleSceneProvider


ROOT = Path(__file__).resolve().parents[1]


def load_config():
    config_root = ROOT / "env_cfg" / "task_config"
    with (config_root / "demo_clean.yml").open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    with (config_root / "_embodiment_config.yml").open(encoding="utf-8") as file:
        embodiment = yaml.safe_load(file)
    robot_file = embodiment[config["embodiment"][0]]["file_path"]
    with (Path(robot_file) / "config.yml").open(encoding="utf-8") as file:
        robot_config = yaml.safe_load(file)
    config.update(
        task_name="place_bread_basket",
        task_config="demo_clean",
        embodiment_name=config["embodiment"][0],
        left_robot_file=robot_file,
        right_robot_file=robot_file,
        left_embodiment_config=robot_config,
        right_embodiment_config=robot_config,
        dual_arm_embodied=True,
        save_data=False,
        collect_data=False,
        need_plan=True,
        render_freq=0,
    )
    return config


def trial(config, seed, sigma_mm):
    task = place_bread_basket()
    result = {"seed": seed, "sigma_mm": sigma_mm, "success": False}
    try:
        task.setup_demo(seed=seed, now_ep_num=0, **config)
        provider = OracleSceneProvider(task, sigma_mm / 1000, seed + 1000003)
        initial_scene = provider.estimate()
        result["initial_scene"] = {
            "breads": [vars(item) for item in initial_scene.breads],
            "basket": vars(initial_scene.basket),
            "robot": vars(initial_scene.robot),
        }
        result["position_error_m"] = {
            "breads": [actor.position_error_m for actor in provider.breads],
            "basket": provider.basket.position_error_m,
        }
        result["branch"] = execute(task, provider)
        result["plan_success"] = bool(task.plan_success)
        result["final_state"] = {
            "bread_xyz": [actor.get_pose().p.tolist() for actor in task.bread],
            "basket_xyz": task.breadbasket.get_pose().p.tolist(),
            "left_gripper_open": bool(task.robot.is_left_gripper_open()),
            "right_gripper_open": bool(task.robot.is_right_gripper_open()),
        }
        result["task_success"] = bool(task.check_success())
        result["joints_legal"], excess = task.planned_joints_legal()
        result["joints_legal"] = bool(result["joints_legal"])
        result["joint_excess_rad"] = float(excess)
        result["success"] = bool(
            result["plan_success"] and result["task_success"] and result["joints_legal"]
        )
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if hasattr(task, "scene"):
            task.close_env()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--sigmas-mm", type=float, nargs="+", default=[0, 5, 10, 20])
    parser.add_argument("--output", type=Path, default=ROOT / "vpc_results" / "oracle.jsonl")
    args = parser.parse_args()
    if any(value < 0 for value in args.sigmas_mm):
        parser.error("sigmas-mm must be nonnegative")
    config = load_config()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        for sigma_mm in args.sigmas_mm:
            for seed in args.seeds:
                result = trial(config, seed, sigma_mm)
                line = json.dumps(result, ensure_ascii=False)
                print(
                    json.dumps({key: result.get(key) for key in
                                ("seed", "sigma_mm", "branch", "plan_success",
                                 "task_success", "joints_legal", "success", "error")
                                if key in result}),
                    flush=True,
                )
                output.write(line + "\n")


if __name__ == "__main__":
    main()
