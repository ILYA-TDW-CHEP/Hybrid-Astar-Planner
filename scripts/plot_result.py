#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


def read_path_csv(path: Path):
    poses = []
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            poses.append((float(row["x"]), float(row["y"]), float(row["theta"])))
    return poses


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot a Hybrid A* result path.")
    parser.add_argument("--path", default="results/path.csv", help="CSV path file")
    parser.add_argument("--result", default="results/result.json", help="JSON result file")
    parser.add_argument("--map", default="results/map.json", help="JSON map file")
    parser.add_argument("--save", default="", help="Optional output image path")
    args = parser.parse_args()

    path_file = Path(args.path)
    result_file = Path(args.result)
    map_file = Path(args.map)

    poses = read_path_csv(path_file)
    if not poses:
        raise SystemExit(f"No poses found in {path_file}")

    stats = {}
    if result_file.exists():
        with result_file.open() as file:
            stats = json.load(file).get("stats", {})

    map_data = {}
    if map_file.exists():
        with map_file.open() as file:
            map_data = json.load(file)

    xs = [pose[0] for pose in poses]
    ys = [pose[1] for pose in poses]

    plt.figure(figsize=(8, 4.5))
    if map_data:
        resolution = float(map_data["resolution"])
        for cell in map_data.get("occupied", []):
            lower_left_x = (int(cell["x"])) * resolution
            lower_left_y = (int(cell["y"])) * resolution
            rectangle = plt.Rectangle(
                (lower_left_x, lower_left_y),
                resolution,
                resolution,
                facecolor="0.2",
                edgecolor="none",
                alpha=0.8,
            )
            plt.gca().add_patch(rectangle)

    plt.plot(xs, ys, marker="o", linewidth=2, markersize=3, label="planned path")
    plt.scatter(xs[0], ys[0], marker="s", s=80, label="start")
    plt.scatter(xs[-1], ys[-1], marker="*", s=120, label="goal")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.xlabel("x, m")
    plt.ylabel("y, m")
    plt.title(
        "Hybrid A* result"
        + (f" | success={stats.get('success')}" if stats else "")
    )
    plt.legend()
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=160)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
