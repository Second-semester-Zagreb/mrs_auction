#!/usr/bin/env python3

import sys
from pathlib import Path
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mrs_auction.auction_def import PrecedenceGraph, TaskSpec

def load_yaml(path: Path):
    return yaml.safe_load(path.read_text())

def calculate_all_layers(tasks_dict):
    relations = [(dep, tid) for tid, t in tasks_dict.items() for dep in t.deps]
    g = PrecedenceGraph(tasks_dict, relations)

    layers, remaining = [], set(tasks_dict)
    while remaining:
        g.update_layers()
        current = set(g.Tf)
        if not current:
            break
        layers.append(sorted(current))
        remaining -= current
        for t in current:
            g.remove_node(t)
    return layers

def plot_tasks(ax, tasks, robots):
    def draw_workspace():
        ax.add_patch(Rectangle((-2, -1), 3.5, 3, lw=1, ec="gray", fc="whitesmoke", alpha=0.5))
        ax.plot(0, 0, "ro", ms=8)

    def draw_formation(x, y, formation, n, offset=0.6):
        if formation == "horizontal" and n == 2:
            ax.plot([x, x + offset], [y, y], "k-", lw=2)
            ax.plot([x, x + offset], [y, y], "ko", ms=5, mfc="black")
        elif formation == "vertical" and n == 2:
            ax.plot([x, x], [y, y + offset], "k-", lw=2)
            ax.plot([x, x], [y, y + offset], "ko", ms=5, mfc="black")
        elif formation == "triangle" and n == 3:
            h = offset * 0.866
            ax.plot([x - offset, x + offset, x, x - offset],
                    [y - h / 2, y - h / 2, y + h, y - h / 2], "k-", lw=2)
        elif formation == "square" and n == 4:
            ax.plot([x - offset, x + offset, x + offset, x - offset, x - offset],
                    [y - offset, y - offset, y + offset, y + offset, y - offset], "k-", lw=2)

    def draw_takeoff_and_robots():
        xs = [r["start_location"][0] for r in robots]
        ys = [r["start_location"][1] for r in robots]
        min_rx, max_rx = min(xs) - 0.1, max(xs) + 0.1
        min_ry, max_ry = min(ys) - 0.1, max(ys) + 0.1

        ax.add_patch(Rectangle((min_rx, min_ry), max_rx - min_rx, max_ry - min_ry,
                               lw=2, ec="blue", fc="lightblue", alpha=0.3, hatch="///"))
        ax.text(min_rx, min_ry - 0.1, "Takeoff area", fontsize=9, color="blue", style="italic")

        for r in robots:
            x, y = r["start_location"]
            ax.add_patch(plt.Circle((x, y), 0.03, color="black", fill=False, lw=2))
            ax.plot([x - 0.05, x + 0.05], [y - 0.05, y + 0.05], "k-", lw=1.5)
            ax.plot([x - 0.05, x + 0.05], [y + 0.05, y - 0.05], "k-", lw=1.5)

    draw_workspace()

    for t in tasks:
        x, y = t["location"]
        tid, kind = t["id"], t["type"]

        ax.plot(x, y, "kx", ms=7, mew=2.5)
        ax.text(x, y - 0.1, tid, ha="center", va="top", fontsize=11, fontweight="bold")

        if kind == "MR":
            draw_formation(x, y, t.get("formation", ""), t.get("numb_robots", 1))

    draw_takeoff_and_robots()

    ax.set(xlabel="X Position (m)", ylabel="Y Position (m)", title="(a) Task locations",
           xlim=(-2.5, 2), ylim=(-1.5, 2.5))
    ax.set_aspect("equal")
    ax.title.set_fontsize(14)
    ax.title.set_fontweight("bold")
    for s in ax.spines.values():
        s.set_linewidth(2)


def plot_precedence(ax, tasks):
    def task_num(tid: str) -> int:
        digits = "".join(ch for ch in tid if ch.isdigit())
        return int(digits) if digits else 0

    tasks_dict = {
        t["id"]: TaskSpec(
            task_id=t["id"],
            location=tuple(t["location"]),
            duration_s=t["duration_s"],
            kind=t["type"],
            robots_required=t.get("numb_robots", 1),
            formation=t.get("formation"),
            deps=t.get("deps", []),
        )
        for t in tasks
    }

    layers = calculate_all_layers(tasks_dict)

    layer_w, r, vspace = 2.5, 0.3, 1.2
    labels = ["TF", "TL", "TH", "TI", "TJ", "TK"]

    max_layer = max((len(L) for L in layers), default=1)
    total_h = max_layer * vspace
    graph_w = len(layers) * layer_w
    x_off = (10 - graph_w) / 2

    pos = {}
    for li, layer in enumerate(layers):
        x = li * layer_w + layer_w / 2 - 0.5 + x_off
        y0 = (max_layer - len(layer)) * vspace / 2
        for i, tid in enumerate(sorted(layer, key=task_num)):
            y = total_h - (y0 + i * vspace) - 0.7
            pos[tid] = (x, y)

    # separators + labels
    for li in range(len(layers) + 1):
        x = li * layer_w - 0.5 + x_off
        ax.axvline(x, color="gray", ls="--", lw=1.5, alpha=0.6)
        if li < len(layers):
            ax.text(x + layer_w / 2, total_h + 0.8,
                    labels[li] if li < len(labels) else f"T{li}",
                    ha="center", va="bottom", fontsize=12, fontweight="bold")

    # deps
    for tid, spec in tasks_dict.items():
        if tid not in pos:
            continue
        cx, cy = pos[tid]
        for dep in spec.deps or []:
            if dep not in pos:
                continue
            dx, dy = pos[dep]
            ax.annotate("", xy=(cx - (r + 0.1), cy), xytext=(dx + (r + 0.1), dy),
                        arrowprops=dict(arrowstyle="->", lw=1.5, color="black", alpha=0.7))

    # nodes
    for tid, (x, y) in pos.items():
        ax.add_patch(plt.Circle((x, y), r + 0.1, fc="white", ec="black", lw=2, zorder=10))
        ax.text(x, y, tid, ha="center", va="center", fontsize=11, fontweight="bold", zorder=11)

    ax.set_aspect("equal")
    ax.set(xlim=(-1, 9), ylim=(-1, total_h + 1.5), title="(b) Task decomposition")
    ax.set_xticks([]); ax.set_yticks([])
    ax.title.set_fontsize(14)
    ax.title.set_fontweight("bold")
    for s in ax.spines.values():
        s.set_linewidth(2)

if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    config_dir = script_dir.parent / "config"

    tasks_file = (config_dir / "custom_tasks.yaml")
    tasks_file = tasks_file if tasks_file.exists() else (config_dir / "tasks.yaml")

    tasks = load_yaml(tasks_file).get("tasks", [])
    robots = load_yaml(config_dir / "robots.yaml").get("robots", [])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    plot_tasks(ax1, tasks, robots)
    plot_precedence(ax2, tasks)
    plt.tight_layout()
    plt.show()
