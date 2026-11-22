"""Export a completed vanilla RRT tree without changing planning or sampling."""

import os
from pathlib import Path
import tempfile

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.collections import LineCollection

from planning.plotting import label_plot, prepare_output_path


GIF_FPS = 10
MAX_GROWTH_FRAMES = 100
FINAL_HOLD_FRAMES = 15


def tree_segments(tree):
    """Vertex IDs preserve insertion order; vanilla RRT never rewires edges."""
    return np.asarray([
        [np.asarray(tree.vertices[tree.edges[child]]).reshape(-1)[:2],
         np.asarray(tree.vertices[child]).reshape(-1)[:2]]
        for child in sorted(tree.edges)
    ], dtype=float).reshape((-1, 2, 2))


def growth_counts(edge_count):
    """Thin pictures, not the tree: the final picture includes every edge."""
    return np.unique(np.linspace(0, edge_count,
                                min(MAX_GROWTH_FRAMES, edge_count + 1), dtype=int))


def export_rrt(planner, plan, image_path=None, gif_path=None):
    """Save an ordered growth replay and/or the same final scene as a PNG.

    Replay speed is independent of planning time. Failed searches retain their
    explored tree; identical endpoints and empty trees are valid still scenes.
    """
    if image_path is None and gif_path is None:
        return
    if image_path is not None:
        image_path = prepare_output_path(image_path, '.png')
    if gif_path is not None:
        gif_path = prepare_output_path(gif_path, '.gif')

    segments = tree_segments(planner.tree)
    figure, axes = plt.subplots(figsize=(6.4, 4.8), dpi=100)
    temporary_path = None
    try:
        label_plot(figure, planner.netids)
        axes.imshow(planner.map, cmap='gray', origin='lower', interpolation='none',
                    vmin=0, vmax=1, extent=planner.prob.extents.ravel()[:4])
        axes.set_xlim(planner.prob.extents[0])
        axes.set_ylim(planner.prob.extents[1])
        for point, color in ((planner.start, 'g'), (planner.end, 'r')):
            point = np.asarray(point).reshape(-1)
            if len(point) >= 2 and np.isfinite(point[:2]).all():
                axes.scatter(point[0], point[1], c=color, zorder=4)
        edges = LineCollection([], colors='red', linewidths=0.8, zorder=2)
        axes.add_collection(edges)
        route, = axes.plot([], [], color='blue', linewidth=2, zorder=3)
        shortcut, = axes.plot([], [], color='green', linewidth=2, zorder=3)
        status = axes.text(0.02, 0.02, '', transform=axes.transAxes,
                           bbox={'facecolor': 'white', 'alpha': 0.8}, zorder=5)

        def final_scene():
            edges.set_segments(segments)
            if plan is not None:
                points = np.asarray(plan).reshape((-1, 2))
                route.set_data(points[:, 0], points[:, 1])
                if planner.shorten_path is not None:
                    points = np.asarray(planner.shorten_path).reshape((-1, 2))
                    shortcut.set_data(points[:, 0], points[:, 1])
            status.set_text('Path found' if plan is not None else 'No path found')

        if gif_path is not None:
            # Pillow retains frames in memory, so cap frame count and resolution.
            writer = PillowWriter(fps=GIF_FPS)
            with tempfile.NamedTemporaryFile(dir=str(gif_path.parent), suffix='.gif',
                                             delete=False) as temporary:
                temporary_path = Path(temporary.name)
            with writer.saving(figure, str(temporary_path), dpi=100):
                for count in growth_counts(len(segments)):
                    edges.set_segments(segments[:count])
                    status.set_text('Tree edges: {} / {}'.format(count, len(segments)))
                    writer.grab_frame()
                final_scene()
                for _ in range(FINAL_HOLD_FRAMES):
                    writer.grab_frame()
            os.replace(str(temporary_path), str(gif_path))
            temporary_path = None
            print('Saved RRT animation to', gif_path)
        else:
            final_scene()
        if image_path is not None:
            figure.savefig(str(image_path), format='png', dpi=150, bbox_inches='tight')
            print('Saved RRT image to', image_path)
    finally:
        plt.close(figure)
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
