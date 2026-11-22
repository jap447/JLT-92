#!/usr/bin/env python3
"""Regression checks for headless PNG/GIF output and actual CLI wiring."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
import numpy as np
from PIL import Image

from planning.plotting import add_export_arguments, configure_exports, prepare_output_path
from planning.rrt_export import export_rrt, tree_segments, growth_counts, MAX_GROWTH_FRAMES


class TestRRTExport(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.output = Path(temporary.name)
        vertices = [np.array([[x, y]], dtype=float)
                    for x, y in [(1, 1), (1, 3), (3, 3), (4, 4)]]
        # Scrambled dictionary order ensures replay uses vertex insertion order.
        tree = SimpleNamespace(vertices=vertices, edges={3: 2, 1: 0, 2: 1})
        self.planner = SimpleNamespace(
            map=np.ones((6, 6), dtype=bool), tree=tree, netids=('abc123', 'xyz789'),
            prob=SimpleNamespace(extents=np.array([[0, 6], [0, 6]])),
            start=vertices[0], end=vertices[-1], shorten_path=None,
        )
        self.path = np.vstack(vertices)

    def test_order_and_frame_limit(self):
        segments = tree_segments(self.planner.tree)
        np.testing.assert_array_equal(segments[:, 1], self.path[1:])
        counts = growth_counts(100000)
        self.assertLessEqual(len(counts), MAX_GROWTH_FRAMES)
        self.assertEqual(counts[0], 0)
        self.assertEqual(counts[-1], 100000)
        self.assertTrue(np.all(np.diff(counts) > 0))

    def test_gif_grows_and_final_png_preserves_labels_and_path(self):
        image_path, gif_path = self.output / 'nested/final.png', self.output / 'tree.gif'
        original = self.path.copy()
        savefig = Figure.savefig
        titles = []

        def inspect_save(figure, *args, **kwargs):
            titles.append(figure._suptitle.get_text())
            return savefig(figure, *args, **kwargs)

        with patch.object(plt, 'show') as show, patch.object(Figure, 'savefig', inspect_save):
            export_rrt(self.planner, self.path, image_path, gif_path)
        show.assert_not_called()
        self.assertTrue(all(title == 'NetIDs: abc123, xyz789' for title in titles))
        np.testing.assert_array_equal(self.path, original)
        with Image.open(image_path) as image:
            self.assertEqual(image.format, 'PNG')
        with Image.open(gif_path) as gif:
            self.assertEqual(gif.size, (640, 480))
            self.assertGreater(gif.n_frames, 2)
            frames = []
            duration = 0
            for index in range(gif.n_frames):
                gif.seek(index)
                frames.append(np.asarray(gif.convert('RGB')).copy())
                duration += gif.info['duration']
            red = lambda f: ((f[:, :, 0] > 180) & (f[:, :, 1] < 120) & (f[:, :, 2] < 120)).sum()
            blue = lambda f: ((f[:, :, 2] > 180) & (f[:, :, 0] < 120) & (f[:, :, 1] < 120)).sum()
            self.assertGreater(red(frames[-2]), red(frames[0]))
            self.assertGreater(blue(frames[-1]), blue(frames[0]))
            self.assertGreaterEqual(duration, 1500)
        self.assertEqual(plt.get_fignums(), [])

    def test_shortcut_failure_and_identical_endpoints(self):
        for mode in ('shortcut', 'failure', 'identical'):
            with self.subTest(mode=mode):
                plan = self.path
                if mode == 'shortcut':
                    self.planner.shorten_path = self.path[[0, -1]]
                else:
                    self.planner.shorten_path = None
                if mode == 'failure':
                    plan = None
                elif mode == 'identical':
                    self.planner.end = self.planner.start.copy()
                    self.planner.tree = SimpleNamespace(vertices=[self.planner.start], edges={})
                    plan = self.planner.start.copy()
                export_rrt(self.planner, plan, self.output / (mode + '.png'),
                           self.output / (mode + '.gif'))
                with Image.open(self.output / (mode + '.gif')) as gif:
                    self.assertGreaterEqual(gif.n_frames, 1)
        self.assertEqual(plt.get_fignums(), [])

    def test_output_errors_and_gif_cleanup(self):
        with self.assertRaises(ValueError):
            prepare_output_path(self.output / 'wrong.jpg', '.png')
        folder = self.output / 'directory.png'
        folder.mkdir()
        with self.assertRaises(ValueError):
            prepare_output_path(folder, '.png')
        with patch('planning.rrt_export.PillowWriter.saving', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                export_rrt(self.planner, self.path, gif_path=self.output / 'failed.gif')
        self.assertEqual(list(self.output.glob('*.gif')), [])
        self.assertEqual(plt.get_fignums(), [])

    def test_no_show_configuration(self):
        parser = argparse.ArgumentParser()
        add_export_arguments(parser, rrt=True)
        args = parser.parse_args(['--no-show', '--save-image', str(self.output / 'image.png')])
        configure_exports(args, parser)
        self.assertEqual(matplotlib.get_backend().lower(), 'agg')


class TestExportCLI(unittest.TestCase):
    def setUp(self):
        self.package = Path(__file__).resolve().parents[1]
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.map = self.directory / 'map.txt'
        np.savetxt(self.map, np.ones((6, 6), dtype=int), fmt='%d')
        self.environment = os.environ.copy()
        self.environment['PYTHONPATH'] = str(self.package / 'src') + os.pathsep + self.environment.get('PYTHONPATH', '')
        self.environment['MPLBACKEND'] = 'TkAgg'
        self.environment.pop('DISPLAY', None)
        self.environment.pop('WAYLAND_DISPLAY', None)

    def run_cli(self, script, arguments):
        return subprocess.run([sys.executable, str(self.package / 'scripts' / script),
                               '--netids', 'abc123', *arguments],
                              cwd=str(self.directory), env=self.environment,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              universal_newlines=True, timeout=60)

    def test_rrt_headless_exports_and_unchanged_plan(self):
        args = ['-m', str(self.map), '-s', '1', '1', '-g', '4', '4', '-b', '1', '--no-show']
        baseline = self.run_cli('run_rrt', args)
        result = self.run_cli('run_rrt', args + ['--show-tree', '--shortcut',
                              '--save-image', 'writeup/rrt.png', '--save-gif', 'writeup/rrt.gif'])
        self.assertEqual(baseline.returncode, 0, baseline.stdout)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(baseline.stdout.split('Plan:')[1].split('Planning Time:')[0],
                         result.stdout.split('Plan:')[1].split('Planning Time:')[0])
        for name in ('rrt.png', 'rrt.gif'):
            self.assertTrue((self.directory / 'writeup' / name).is_file())

    def test_rrt_failure_still_exports_and_exits_nonzero(self):
        result = self.run_cli('run_rrt', ['-m', str(self.map), '-s', '-1', '1', '-g', '4', '4',
                              '--no-show', '--save-image', 'writeup/failed.png',
                              '--save-gif', 'writeup/failed.gif'])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('RRT could not find a valid path', result.stdout)
        self.assertTrue((self.directory / 'writeup/failed.gif').is_file())

    def test_astar_r2_and_se2_headless_images(self):
        for mode, n, radius, start, goal, extra in (
                ('r2', '25', '3', ['1', '6'], ['9', '3'], []),
                ('se2', '40', '4', ['1', '6', '0'], ['9', '3', '45'], ['-c', '3'])):
            with self.subTest(mode=mode):
                result = self.run_cli('run_astar', ['-m', str(self.package / 'test/share/map1.txt'),
                    '-n', n, '-r', radius, '--show-edges', '--no-show',
                    '--save-image', 'writeup/' + mode + '.png', mode, '-s', *start, '-g', *goal, *extra])
                self.assertEqual(result.returncode, 0, result.stdout)
                with Image.open(self.directory / 'writeup' / (mode + '.png')) as image:
                    self.assertEqual(image.format, 'PNG')

    def test_invalid_filename_fails_before_planning(self):
        result = self.run_cli('run_rrt', ['-s', '1', '1', '-g', '4', '4',
                              '--no-show', '--save-gif', 'writeup/not-a-gif.png'])
        self.assertEqual(result.returncode, 2)
        self.assertIn('must end in .gif', result.stdout)
        self.assertNotIn('Planning complete', result.stdout)

    def test_astar_no_path_exits_nonzero_without_image(self):
        result = self.run_cli('run_astar', ['-m', str(self.map), '-n', '9', '-r', '0.1',
                              '--no-show', '--save-image', 'writeup/no-path.png',
                              'r2', '-s', '1', '1', '-g', '4', '4'])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not reachable', result.stdout)
        self.assertFalse((self.directory / 'writeup/no-path.png').exists())


if __name__ == '__main__':
    unittest.main()
