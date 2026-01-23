#!/usr/bin/env python3
"""Visualize tasks and robots (spatial) and precedence graph side-by-side."""

import sys
from pathlib import Path
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Import project modules
sys.path.insert(0, str(Path(__file__).parent.parent))
from mrs_auction.auction_def import PrecedenceGraph, TaskSpec


def load_yaml(filepath):
    with open(filepath, "r") as f:
        return yaml.safe_load(f)


def calculate_all_layers(tasks_dict):
    relations = []
    for tid, task in tasks_dict.items():
        for dep in task.deps:
            relations.append((dep, tid))

    graph = PrecedenceGraph(tasks_dict, relations)
    layers = []
    remaining = set(tasks_dict.keys())

    while remaining:
        graph.update_layers()
        current = set(graph.Tf)
        if not current:
            break
        layers.append(sorted(current))
        remaining -= current
        for t in current:
            graph.remove_node(t)

    return layers


def plot_tasks(ax, tasks, robots):
    """Plot spatial task/robot view on given axis."""
    # Working space box (example size to match sample look)
    ws_rect = Rectangle((-2, -1), 3.5, 3, linewidth=1, edgecolor="gray",
                        facecolor="whitesmoke", alpha=0.5)
    ax.add_patch(ws_rect)

    # Origin marker
    ax.plot(0, 0, "ro", markersize=8)

    # Tasks
    for task in tasks:
        x, y = task["location"]
        task_type = task["type"]
        task_id = task["id"]

        ax.plot(x, y, "kx", markersize=7, markeredgewidth=2.5)
        ax.text(x, y - 0.1, task_id, ha="center", va="top", fontsize=11, fontweight="bold")

        if task_type == "MR":
            formation = task.get("formation", "")
            offset = 0.6
            if formation == "horizontal" and task["numb_robots"] == 2:
                ax.plot([x, x + offset], [y, y], "k-", linewidth=2)
                ax.plot([x, x + offset], [y, y], "ko", markersize=5, markerfacecolor="black")
            elif formation == "vertical" and task["numb_robots"] == 2:
                ax.plot([x, x], [y, y + offset], "k-", linewidth=2)
                ax.plot([x, x], [y, y + offset], "ko", markersize=5, markerfacecolor="black")
            elif formation == "triangle" and task["numb_robots"] == 3:
                h = offset * 0.866
                tx = [x - offset, x + offset, x, x - offset]
                ty = [y - h / 2, y - h / 2, y + h, y - h / 2]
                ax.plot(tx, ty, "k-", linewidth=2)
            elif formation == "square" and task["numb_robots"] == 4:
                sx = [x - offset, x + offset, x + offset, x - offset, x - offset]
                sy = [y - offset, y - offset, y + offset, y + offset, y - offset]
                ax.plot(sx, sy, "k-", linewidth=2)

    # Robots + takeoff area
    robot_xs = [r["start_location"][0] for r in robots]
    robot_ys = [r["start_location"][1] for r in robots]
    min_rx, max_rx = min(robot_xs) - 0.1, max(robot_xs) + 0.1
    min_ry, max_ry = min(robot_ys) - 0.1, max(robot_ys) + 0.1
    takeoff_rect = Rectangle((min_rx, min_ry), max_rx - min_rx, max_ry - min_ry,
                             linewidth=2, edgecolor="blue", facecolor="lightblue",
                             alpha=0.3, hatch="///")
    ax.add_patch(takeoff_rect)
    ax.text(min_rx, min_ry - 0.1, "Takeoff area", fontsize=9, color="blue", style="italic")

    for robot in robots:
        x, y = robot["start_location"]
        ax.add_patch(plt.Circle((x, y), 0.03, color="black", fill=False, linewidth=2))
        ax.plot([x - 0.05, x + 0.05], [y - 0.05, y + 0.05], "k-", linewidth=1.5)
        ax.plot([x - 0.05, x + 0.05], [y + 0.05, y - 0.05], "k-", linewidth=1.5)

    ax.set_xlabel("X Position (m)", fontsize=12)
    ax.set_ylabel("Y Position (m)", fontsize=12)
    ax.set_title("(a) Task locations", fontsize=14, fontweight="bold")
    ax.set_aspect("equal")
    ax.set_xlim(-2.5, 2)
    ax.set_ylim(-1.5, 2.5)
    for spine in ax.spines.values():
        spine.set_linewidth(2)


def plot_precedence(ax, tasks):
    """Plot precedence layers on given axis."""
    # Build TaskSpec dict
    tasks_dict = {}
    for t in tasks:
        tid = t["id"]
        tasks_dict[tid] = TaskSpec(
            task_id=tid,
            location=(t["location"][0], t["location"][1]),
            duration_s=t["duration_s"],
            kind=t["type"],
            robots_required=t.get("numb_robots", 1),
            formation=t.get("formation"),
            deps=t.get("deps", []),
        )

    layers = calculate_all_layers(tasks_dict)

    layer_width = 2.5
    circle_radius = 0.3
    vertical_spacing = 1.2
    layer_labels = ["TF", "TL", "TH", "TI", "TJ", "TK"]

    max_layer_size = max(len(layer) for layer in layers) if layers else 1
    total_height = max_layer_size * vertical_spacing
    graph_width = len(layers) * layer_width
    x_center_offset = (10 - graph_width) / 2  # Center within 10-unit width

    task_positions = {}
    for layer_idx, layer_tasks in enumerate(layers):
        x = layer_idx * layer_width + layer_width / 2 - 0.5 + x_center_offset
        y_start = (max_layer_size - len(layer_tasks)) * vertical_spacing / 2

        # Sort by task number so placement is in numeric order top-to-bottom
        def task_num(tid: str) -> int:
            digits = ''.join(ch for ch in tid if ch.isdigit())
            return int(digits) if digits else 0

        sorted_layer = sorted(layer_tasks, key=task_num)

        for idx, tid in enumerate(sorted_layer):
            y = total_height - (y_start + idx * vertical_spacing) - 0.7
            task_positions[tid] = (x, y)

    # Layer separators/labels
    for layer_idx in range(len(layers) + 1):
        x = layer_idx * layer_width - 0.5 + x_center_offset
        ax.axvline(x, color="gray", linestyle="--", linewidth=1.5, alpha=0.6)
        if layer_idx < len(layers):
            label = layer_labels[layer_idx] if layer_idx < len(layer_labels) else f"T{layer_idx}"
            ax.text(x + layer_width / 2, total_height + 0.8, label,
                    ha="center", va="bottom", fontsize=12, fontweight="bold")

    # Dependencies
    for tid, spec in tasks_dict.items():
        if spec.deps and tid in task_positions:
            curr_x, curr_y = task_positions[tid]
            for dep_id in spec.deps:
                if dep_id in task_positions:
                    dep_x, dep_y = task_positions[dep_id]
                    ax.annotate("", xy=(curr_x - (circle_radius + 0.1), curr_y),
                                xytext=(dep_x + (circle_radius + 0.1), dep_y),
                                arrowprops=dict(arrowstyle="->", lw=1.5, color="black", alpha=0.7))

    # Circles
    for tid, (x, y) in task_positions.items():
        ax.add_patch(plt.Circle((x, y), circle_radius + 0.1, facecolor="white", edgecolor="black", linewidth=2, zorder=10))
        ax.text(x, y, tid, ha="center", va="center", fontsize=11, fontweight="bold", zorder=11)

    ax.set_aspect("equal")
    ax.set_xlim(-1, 9)
    ax.set_ylim(-1, total_height + 1.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("(b) Task decomposition", fontsize=14, fontweight="bold")
    for spine in ax.spines.values():
        spine.set_linewidth(2)


def main():
    script_dir = Path(__file__).parent
    config_dir = script_dir.parent / "config"

    # Prefer custom_tasks.yaml if present, else fallback to tasks.yaml
    custom_tasks = config_dir / "custom_tasks.yaml"
    tasks_file = custom_tasks if custom_tasks.exists() else config_dir / "tasks.yaml"
    robots_file = config_dir / "robots.yaml"

    tasks_data = load_yaml(tasks_file)
    robots_data = load_yaml(robots_file)
    tasks = tasks_data.get("tasks", [])
    robots = robots_data.get("robots", [])

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(12, 6))

    plot_tasks(ax_left, tasks, robots)
    plot_precedence(ax_right, tasks)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
