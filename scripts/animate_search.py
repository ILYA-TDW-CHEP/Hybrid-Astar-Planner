#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


def load_json(path: Path):
    with path.open() as file:
        return json.load(file)


def main() -> int:
    parser = argparse.ArgumentParser(description="Animate Hybrid A* search expansion.")
    parser.add_argument("--result", default="results/result.json", help="JSON result file")
    parser.add_argument("--map", default="results/map.json", help="JSON map file")
    parser.add_argument("--output", default="results/search.gif", help="Output GIF path")
    parser.add_argument("--steps-per-frame", type=int, default=60, help="Expanded nodes revealed per frame")
    parser.add_argument("--interval", type=int, default=80, help="Frame interval in ms")
    parser.add_argument("--hold-final-ms", type=int, default=1000, help="How long to hold the final frame")
    parser.add_argument(
        "--obstacle-reveal-radius",
        type=int,
        default=1,
        help="Reveal occupied cells only when they are this close to explored cells",
    )
    args = parser.parse_args()

    result = load_json(Path(args.result))
    map_data = load_json(Path(args.map))

    width = int(map_data["width"])
    height = int(map_data["height"])
    resolution = float(map_data["resolution"])
    occupied = map_data.get("occupied", [])
    expanded = result.get("expanded", [])
    path = result.get("path", [])

    if not expanded:
        raise SystemExit("No expanded nodes in result JSON. Re-run ./build/hybrid_astar_app first.")

    steps_per_frame = max(1, args.steps_per_frame)
    search_frame_count = max(1, (len(expanded) + steps_per_frame - 1) // steps_per_frame)
    hold_frame_count = max(0, (args.hold_final_ms + args.interval - 1) // args.interval)
    frame_count = search_frame_count + hold_frame_count

    obstacle_mask = np.zeros((height, width), dtype=bool)
    for cell in occupied:
        obstacle_mask[int(cell["y"]), int(cell["x"])] = True

    max_cost = max(float(node["g_cost"]) for node in expanded) or 1.0
    gradient = plt.get_cmap("viridis")

    fig, ax = plt.subplots(figsize=(10, 5.5))
    image = ax.imshow(
        np.full((height, width, 3), 0.68, dtype=float),
        origin="lower",
        extent=(0, width * resolution, 0, height * resolution),
        interpolation="nearest",
    )

    if path:
        start = path[0]
        goal = path[-1]
        ax.scatter(start["x"], start["y"], marker="s", s=90, c="lime", edgecolors="black", label="start", zorder=4)
        ax.scatter(goal["x"], goal["y"], marker="*", s=160, c="red", edgecolors="black", label="goal", zorder=4)

    path_line, = ax.plot([], [], color="white", linewidth=2.0, label="final path", zorder=3)
    ax.set_aspect("equal")
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.legend(loc="upper right")
    title = ax.set_title("Hybrid A* search")

    def update(frame_index):
        search_frame_index = min(frame_index, search_frame_count - 1)
        visible_count = min(len(expanded), (search_frame_index + 1) * steps_per_frame)
        current = np.full((height, width, 3), 0.68, dtype=float)
        explored_mask = np.zeros((height, width), dtype=bool)

        for node in expanded[:visible_count]:
            x = int(node["grid_x"])
            y = int(node["grid_y"])
            if not (0 <= x < width and 0 <= y < height):
                continue

            explored_mask[y, x] = True
            if not obstacle_mask[y, x]:
                color = gradient(float(node["g_cost"]) / max_cost)[:3]
                current[y, x] = color

        reveal_radius = max(0, args.obstacle_reveal_radius)
        revealed_obstacles = np.zeros((height, width), dtype=bool)
        explored_y, explored_x = np.nonzero(explored_mask)
        for y, x in zip(explored_y, explored_x):
            y0 = max(0, y - reveal_radius)
            y1 = min(height, y + reveal_radius + 1)
            x0 = max(0, x - reveal_radius)
            x1 = min(width, x + reveal_radius + 1)
            revealed_obstacles[y0:y1, x0:x1] |= obstacle_mask[y0:y1, x0:x1]

        current[revealed_obstacles] = 0.0

        image.set_data(current)

        if visible_count == len(expanded) and path:
            path_line.set_data([p["x"] for p in path], [p["y"] for p in path])

        title.set_text(f"Hybrid A* search | expanded {visible_count}/{len(expanded)}")
        return image, path_line, title

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=frame_count,
        interval=args.interval,
        blit=False,
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    anim.save(output, writer=animation.PillowWriter(fps=max(1, 1000 // args.interval)))
    plt.close(fig)
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
