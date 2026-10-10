"""Replay one oracle plan to separate planning variation from execution variation.

Example: python -m scripts.vpc_replay --seed 2 --sigma-mm 5 --replays 3
"""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from envs.place_bread_basket import place_bread_basket
from scripts.vpc_oracle import ROOT, load_config
from vpc.place_bread_basket import execute
from vpc.scene import OracleSceneProvider


def path_summary(paths):
    return [
        {
            "status": path.get("status") if isinstance(path, dict) else None,
            "steps": int(np.asarray(path["position"]).shape[0])
            if isinstance(path, dict) and path.get("position") is not None else 0,
            "sha256": hashlib.sha256(np.asarray(path["position"]).tobytes()).hexdigest()
            if isinstance(path, dict) and path.get("position") is not None else None,
        }
        for path in paths
    ]


def run(config, seed, sigma_mm, noise_target, noise_phase, planned_paths=None):
    task = place_bread_basket()
    replay = planned_paths is not None
    result = {"mode": "replay" if replay else "plan", "success": False, "stages": []}
    paths = None
    try:
        setup_config = dict(config)
        setup_config["need_plan"] = not replay
        if replay:
            setup_config["left_joint_path"] = deepcopy(planned_paths["left"])
            setup_config["right_joint_path"] = deepcopy(planned_paths["right"])
        task.setup_demo(seed=seed, now_ep_num=0, **setup_config)
        provider = OracleSceneProvider(task, sigma_mm / 1000, seed + 1000003,
                                       noise_target=noise_target, noise_phase=noise_phase)

        def record_stage(name):
            result["stages"].append({
                "name": name,
                "bread_xyz": [actor.get_pose().p.tolist() for actor in task.bread],
                "basket_xyz": task.breadbasket.get_pose().p.tolist(),
            })

        result["branch"] = execute(task, provider, on_stage=record_stage)
        result["plan_success"] = bool(task.plan_success)
        result["task_success"] = bool(task.check_success())
        result["joints_legal"], _ = task.planned_joints_legal()
        result["joints_legal"] = bool(result["joints_legal"])
        result["success"] = bool(
            result["plan_success"] and result["task_success"] and result["joints_legal"]
        )
        result["path_summary"] = {
            "left": path_summary(task.left_joint_path),
            "right": path_summary(task.right_joint_path),
        }
        if replay:
            result["paths_consumed"] = {
                "left": task.left_cnt,
                "right": task.right_cnt,
            }
            result["all_paths_consumed"] = bool(
                task.left_cnt == len(planned_paths["left"])
                and task.right_cnt == len(planned_paths["right"])
            )
        else:
            paths = {
                "left": deepcopy(task.left_joint_path),
                "right": deepcopy(task.right_joint_path),
            }
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if hasattr(task, "scene"):
            task.close_env()
    return result, paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--sigma-mm", type=float, required=True)
    parser.add_argument("--noise-target", choices=["all", "bread", "basket", "none"], default="all")
    parser.add_argument("--noise-phase", choices=["all", "grasp", "place", "none"], default="all")
    parser.add_argument("--replays", type=int, default=3)
    parser.add_argument("--output", type=Path, default=ROOT / "vpc_results" / "replay.jsonl")
    args = parser.parse_args()
    if args.sigma_mm < 0 or args.replays < 1:
        parser.error("sigma-mm must be nonnegative and replays must be positive")
    config = load_config()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    baseline, paths = run(config, args.seed, args.sigma_mm, args.noise_target,
                          args.noise_phase)
    with args.output.open("w", encoding="utf-8") as output:
        output.write(json.dumps(baseline) + "\n")
        print(json.dumps({key: baseline.get(key) for key in
                          ("mode", "success", "plan_success", "task_success", "error")
                          if key in baseline}), flush=True)
        if not baseline.get("plan_success") or paths is None:
            return
        for repeat_index in range(args.replays):
            result, _ = run(config, args.seed, args.sigma_mm, args.noise_target,
                            args.noise_phase, planned_paths=paths)
            result["repeat_index"] = repeat_index
            output.write(json.dumps(result) + "\n")
            print(json.dumps({key: result.get(key) for key in
                              ("mode", "repeat_index", "success", "plan_success",
                               "task_success", "all_paths_consumed", "error")
                              if key in result}), flush=True)


if __name__ == "__main__":
    main()
