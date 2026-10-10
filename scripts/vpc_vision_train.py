"""Train a small RGB-only bread/basket segmentation baseline.

Example: python -m scripts.vpc_vision_train --manifest-dir vpc_results/vision_head_split
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.nn import functional as F


CLASSES = ("background", "bread", "basket")


def block(channels_in, channels_out):
    return nn.Sequential(
        nn.Conv2d(channels_in, channels_out, 3, padding=1), nn.ReLU(inplace=True),
        nn.Conv2d(channels_out, channels_out, 3, padding=1), nn.ReLU(inplace=True),
    )


class SmallUNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc1 = block(3, 16)
        self.enc2 = block(16, 32)
        self.middle = block(32, 64)
        self.dec2 = block(96, 32)
        self.dec1 = block(48, 16)
        self.output = nn.Conv2d(16, len(CLASSES), 1)

    def forward(self, image):
        first = self.enc1(image)
        second = self.enc2(F.max_pool2d(first, 2))
        middle = self.middle(F.max_pool2d(second, 2))
        up_second = F.interpolate(middle, size=second.shape[-2:], mode="bilinear",
                                  align_corners=False)
        decoded_second = self.dec2(torch.cat((up_second, second), dim=1))
        up_first = F.interpolate(decoded_second, size=first.shape[-2:], mode="bilinear",
                                 align_corners=False)
        return self.output(self.dec1(torch.cat((up_first, first), dim=1)))


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def load_sample(row, width, height):
    image_path = Path(row["image"])
    rgb = np.asarray(Image.open(image_path).convert("RGB"))
    geometry_path = image_path.with_name(image_path.name.replace("_rgb.png", "_geometry.npz"))
    with np.load(geometry_path) as geometry:
        ids = geometry["actor_ids"]
    report = json.loads((image_path.parent / "report.json").read_text(encoding="utf-8"))
    objects = report["cameras"][row["camera"]]["objects"]
    semantic = np.zeros(ids.shape, dtype=np.uint8)
    for name, info in objects.items():
        semantic[ids == info["actor_id"]] = 2 if name == "basket" else 1
    rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
    semantic = cv2.resize(semantic, (width, height), interpolation=cv2.INTER_NEAREST)
    image_tensor = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float() / 255.0
    label_tensor = torch.from_numpy(semantic.astype(np.int64))
    return image_tensor, label_tensor


def boxes_from_mask(mask, class_id, min_pixels=12):
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask == class_id).astype(np.uint8), connectivity=8)
    return [tuple(int(v) for v in stats[index, :4]) for index in range(1, count)
            if stats[index, cv2.CC_STAT_AREA] >= min_pixels]


def box_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    overlap = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * \
              max(0, min(ay + ah, by + bh) - max(ay, by))
    union = aw * ah + bw * bh - overlap
    return overlap / union if union else 0.0


def match_boxes(predicted, actual, threshold=0.5):
    pairs = sorted(((box_iou(p, a), pi, ai)
                    for pi, p in enumerate(predicted)
                    for ai, a in enumerate(actual)), reverse=True)
    used_pred, used_actual = set(), set()
    for score, pi, ai in pairs:
        if score < threshold:
            break
        if pi not in used_pred and ai not in used_actual:
            used_pred.add(pi)
            used_actual.add(ai)
    return len(used_pred), len(predicted) - len(used_pred), len(actual) - len(used_actual)


def foreground_dice_loss(logits, labels):
    probabilities = logits.softmax(dim=1)[:, 1:]
    truth = F.one_hot(labels, num_classes=len(CLASSES)).permute(0, 3, 1, 2)[:, 1:]
    truth = truth.to(probabilities.dtype)
    intersection = (probabilities * truth).sum(dim=(0, 2, 3))
    mass = probabilities.sum(dim=(0, 2, 3)) + truth.sum(dim=(0, 2, 3))
    return (1.0 - (2.0 * intersection + 1.0) / (mass + 1.0)).mean()


@torch.no_grad()
def evaluate(model, rows, samples, width, height, device):
    model.eval()
    intersection = np.zeros(len(CLASSES), dtype=np.int64)
    union = np.zeros(len(CLASSES), dtype=np.int64)
    detection = {name: np.zeros(3, dtype=np.int64) for name in CLASSES[1:]}
    predictions = []
    for row, (image, target) in zip(rows, samples):
        logits = model(image.unsqueeze(0).to(device))
        pred = logits.argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        truth = target.numpy()
        for class_id in range(len(CLASSES)):
            intersection[class_id] += np.logical_and(pred == class_id, truth == class_id).sum()
            union[class_id] += np.logical_or(pred == class_id, truth == class_id).sum()
        for class_id, name in enumerate(CLASSES[1:], start=1):
            found = boxes_from_mask(pred, class_id)
            actual = []
            for obj in row["objects"]:
                if obj["category"] != name:
                    continue
                x, y, box_width, box_height = obj["bbox_xywh"]
                actual.append((x * width / row["image_hw"][1],
                               y * height / row["image_hw"][0],
                               box_width * width / row["image_hw"][1],
                               box_height * height / row["image_hw"][0]))
            detection[name] += match_boxes(found, actual)
        predictions.append(pred)
    return {
        "pixel_iou": {name: float(intersection[i] / union[i]) if union[i] else None
                      for i, name in enumerate(CLASSES)},
        "box_iou_0.5": {name: {"tp": int(counts[0]), "fp": int(counts[1]),
                                "fn": int(counts[2]),
                                "precision": float(counts[0] / (counts[0] + counts[1]))
                                if counts[0] + counts[1] else 0.0,
                                "recall": float(counts[0] / (counts[0] + counts[2]))
                                if counts[0] + counts[2] else 0.0}
                        for name, counts in detection.items()},
    }, predictions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--height", type=int, default=120)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--dice-weight", type=float, default=1.0)
    args = parser.parse_args()
    if args.epochs < 1 or args.width % 4 or args.height % 4 or args.dice_weight < 0:
        parser.error("epochs must be positive, image dimensions divisible by 4, dice-weight nonnegative")
    output_dir = args.output_dir or args.manifest_dir / "rgb_baseline"
    output_dir.mkdir(parents=True, exist_ok=True)
    train_rows = read_rows(args.manifest_dir / "train.jsonl")
    val_rows = read_rows(args.manifest_dir / "val.jsonl")
    if {row["seed"] for row in train_rows} & {row["seed"] for row in val_rows}:
        raise ValueError("train/val seed leakage")
    train = [load_sample(row, args.width, args.height) for row in train_rows]
    val = [load_sample(row, args.width, args.height) for row in val_rows]
    counts = np.bincount(np.concatenate([label.numpy().ravel() for _, label in train]),
                         minlength=len(CLASSES))
    weights = np.minimum(8.0, np.sqrt(counts[0] / np.maximum(counts, 1)))
    weights = weights / weights[0]
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SmallUNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_function = nn.CrossEntropyLoss(weight=torch.tensor(weights, dtype=torch.float32,
                                                            device=device))
    generator = np.random.default_rng(args.seed)
    history = []
    for epoch in range(args.epochs):
        model.train()
        losses = []
        for indices in np.array_split(generator.permutation(len(train)),
                                      max(1, (len(train) + 3) // 4)):
            images = torch.stack([train[i][0] for i in indices]).to(device)
            labels = torch.stack([train[i][1] for i in indices]).to(device)
            optimizer.zero_grad()
            logits = model(images)
            loss = loss_function(logits, labels) + \
                   args.dice_weight * foreground_dice_loss(logits, labels)
            loss.backward()
            optimizer.step()
            losses.append(float(loss.item()))
        history.append(float(np.mean(losses)))
        if epoch == 0 or (epoch + 1) % 5 == 0 or epoch + 1 == args.epochs:
            print(json.dumps({"epoch": epoch + 1, "train_loss": history[-1]}), flush=True)
    train_metrics, _ = evaluate(model, train_rows, train, args.width, args.height, device)
    val_metrics, predictions = evaluate(model, val_rows, val, args.width, args.height, device)
    for row, prediction in zip(val_rows, predictions):
        name = f"seed_{row['seed']}_{row['stage']}_{row['camera']}_pred.png"
        Image.fromarray(prediction).save(output_dir / name)
    torch.save({"model": model.state_dict(), "classes": CLASSES,
                "width": args.width, "height": args.height,
                "seed": args.seed, "epochs": args.epochs}, output_dir / "checkpoint.pt")
    result = {"train_images": len(train), "val_images": len(val),
              "train_seeds": sorted({row["seed"] for row in train_rows}),
              "val_seeds": sorted({row["seed"] for row in val_rows}),
              "input_size_wh": [args.width, args.height],
              "dice_weight": args.dice_weight,
              "class_pixel_counts_train": dict(zip(CLASSES, map(int, counts))),
              "class_weights": dict(zip(CLASSES, map(float, weights))),
              "train_loss": history, "train": train_metrics, "val": val_metrics}
    (output_dir / "metrics.json").write_text(json.dumps(result, indent=2),
                                               encoding="utf-8")
    print(json.dumps({"train": train_metrics, "val": val_metrics}), flush=True)


if __name__ == "__main__":
    main()
