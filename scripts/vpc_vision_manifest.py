"""Build seed-disjoint detection manifests from VPC camera annotations.

Example: python -m scripts.vpc_vision_manifest --sources DIR1 DIR2 \
  --val-seeds 2 9 --output-dir vpc_results/vision_head_split
"""

import argparse
from collections import Counter
import json
from pathlib import Path


def validate_annotation(row, source):
    height, width = row["image_hw"]
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image size in {source}: {row['image_hw']}")
    image = source / row["image"]
    if not image.is_file():
        raise FileNotFoundError(image)
    report_path = image.parent / "report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report["seed"] != row["seed"] or report["stage"] != row["stage"]:
        raise ValueError(f"annotation/report mismatch: {image}")
    reported_objects = report["cameras"][row["camera"]]["objects"]
    expected_breads = sum(name.startswith("bread_") for name in reported_objects)
    visible_breads = 0
    seen_instances = set()
    for obj in row["objects"]:
        if obj["category"] not in {"bread", "basket"}:
            raise ValueError(f"unexpected category in {image}: {obj['category']}")
        x, y, box_width, box_height = obj["bbox_xywh"]
        if not (0 <= x < width and 0 <= y < height and
                box_width > 0 and box_height > 0 and
                x + box_width <= width and y + box_height <= height):
            raise ValueError(f"out-of-bounds bbox in {image}: {obj['bbox_xywh']}")
        if obj["visible_pixels"] < 1:
            raise ValueError(f"empty visible object in {image}: {obj}")
        source_object = reported_objects.get(obj["instance"])
        if source_object is None or source_object["pixel_count"] != obj["visible_pixels"]:
            raise ValueError(f"annotation/report object mismatch: {image}, {obj['instance']}")
        if obj["instance"] in seen_instances:
            raise ValueError(f"duplicate instance in {image}: {obj['instance']}")
        if (obj["instance"] == "basket") != (obj["category"] == "basket"):
            raise ValueError(f"category/instance mismatch in {image}: {obj}")
        seen_instances.add(obj["instance"])
        visible_breads += obj["category"] == "bread"
    return {**row, "image": str(image.resolve()),
            "scene_bread_count": expected_breads,
            "unseen_bread_count": expected_breads - visible_breads,
            "plan_success_so_far": report.get("plan_success_so_far")}


def load_annotations(sources, camera):
    rows = []
    seen = set()
    for source in sources:
        source = source.resolve()
        path = source / "annotations.jsonl"
        if not path.is_file():
            raise FileNotFoundError(path)
        for line in path.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            if row["camera"] != camera:
                continue
            key = (row["seed"], row["stage"], row["camera"])
            if key in seen:
                raise ValueError(f"duplicate seed/stage/camera: {key}")
            seen.add(key)
            rows.append(validate_annotation(row, source))
    return sorted(rows, key=lambda row: (row["seed"], row["stage"], row["camera"]))


def summarize(rows):
    return {
        "images": len(rows),
        "seeds": sorted({row["seed"] for row in rows}),
        "stages": dict(sorted(Counter(row["stage"] for row in rows).items())),
        "objects": dict(sorted(Counter(
            obj["category"] for row in rows for obj in row["objects"]
        ).items())),
        "images_with_two_breads_and_basket": sum(
            sum(obj["category"] == "bread" for obj in row["objects"]) >= 2
            and any(obj["category"] == "basket" for obj in row["objects"])
            for row in rows
        ),
        "images_with_one_bread_in_scene": sum(row["scene_bread_count"] == 1 for row in rows),
        "unseen_bread_instances": sum(row["unseen_bread_count"] for row in rows),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", type=Path, nargs="+", required=True)
    parser.add_argument("--val-seeds", type=int, nargs="+", required=True)
    parser.add_argument("--camera", default="head_camera",
                        choices=["head_camera", "front_camera", "left_camera", "right_camera"])
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rows = load_annotations(args.sources, args.camera)
    available = {row["seed"] for row in rows}
    val_seeds = set(args.val_seeds)
    if len(val_seeds) != len(args.val_seeds):
        parser.error("val-seeds contains duplicates")
    if not val_seeds <= available:
        parser.error(f"validation seeds missing from inputs: {sorted(val_seeds - available)}")
    train = [row for row in rows if row["seed"] not in val_seeds]
    val = [row for row in rows if row["seed"] in val_seeds]
    if not train or not val:
        parser.error("both train and validation splits must contain images")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, split in (("train", train), ("val", val)):
        (args.output_dir / f"{name}.jsonl").write_text(
            "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in split),
            encoding="utf-8")
    summary = {"camera": args.camera,
               "sources": [str(path.resolve()) for path in args.sources],
               "train": summarize(train), "val": summarize(val)}
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
