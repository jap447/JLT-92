#!/usr/bin/env python

import multiprocessing
import networkx as nx
import numpy as np
import os
import rosunit
import time
import traceback
import unittest

from planning import search
from planning.samplers import LatticeSampler
from planning.problems import R2Problem
from planning.roadmap import Roadmap


def _rrt_worker(connection, permissible_region, start, goal, planner_factory):
    """Report startup separately so imports do not consume the planning budget."""
    try:
        np.random.seed(112)
        problem = R2Problem(permissible_region)
        planner = planner_factory(problem, permissible_region, batch_size=100)
        connection.send(("ready", None))
        started = time.perf_counter()
        path = planner.Plan(start, goal)
        elapsed = time.perf_counter() - started
        status = "no_path" if path is None else "path"
        connection.send((status, (elapsed, path)))
    except BaseException:
        connection.send(("error", traceback.format_exc()))
    finally:
        connection.close()


def validate_rrt_path(problem, path, start, goal):
    """Reject malformed or geometrically invalid results before awarding credit."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) == 0:
        raise ValueError("RRT must return a nonempty N x 2 path")
    if not np.isfinite(path).all():
        raise ValueError("RRT path coordinates must be finite")
    if not np.allclose(path[0], start) or not np.allclose(path[-1], goal):
        raise ValueError("RRT path must begin at start and end at goal")
    if not problem.check_state_validity(path).all():
        raise ValueError("RRT path contains an invalid state")
    for first, second in zip(path[:-1], path[1:]):
        if not problem.check_edge_validity(first, second):
            raise ValueError("RRT path contains an invalid edge")
    return path


def run_rrt(permissible_region, start, goal, timeout=10.0,
            planner_factory=search.RRTPlanner):
    """Return (path/no_path/timeout/error, detail), always reaping the worker."""
    context = multiprocessing.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    worker = context.Process(
        target=_rrt_worker,
        args=(sender, permissible_region, start, goal, planner_factory),
        daemon=True,
    )
    try:
        worker.start()
        sender.close()
        if not receiver.poll(30.0):
            return "error", "RRT worker did not initialize within 30 seconds"
        status, detail = receiver.recv()
        if status != "ready":
            return "error", detail
        if not receiver.poll(timeout):
            return "timeout", None
        status, detail = receiver.recv()
        if status == "error":
            return status, detail
        elapsed, path = detail
        if elapsed > timeout:
            return "timeout", None
        if status == "path":
            path = validate_rrt_path(R2Problem(permissible_region), path, start, goal)
        return status, path
    except EOFError:
        return "error", "RRT worker exited without reporting a result"
    except Exception:
        return "error", traceback.format_exc()
    finally:
        sender.close()
        receiver.close()
        if worker.pid is not None:
            if worker.is_alive():
                worker.terminate()
            worker.join()
        worker.close()


class TestTradeoff(unittest.TestCase):

    def test_astar_beats_rrt(self):
        
        np.random.seed(112)
        
        fname = os.path.join(os.path.dirname(__file__), "share", "astar_beats_rrt_map.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        problem = R2Problem(permissible_region)
        sampler = LatticeSampler(problem.extents)

        num_vertices = 100
        connection_radius = 3.0
        rm = Roadmap(problem, sampler, num_vertices, connection_radius)
        
        start = np.array([1, 1])
        goal = np.array([19, 19])
        start_id = rm.add_node(start, is_start=True)
        goal_id = rm.add_node(goal, is_start=False)

        try:
            astar = search.ASTARPlanner(rm)
            _ = astar.Plan(start_id, goal_id)
        except nx.NetworkXNoPath as e:
            self.fail("A* failed to find a path")
        
        status, detail = run_rrt(permissible_region, start, goal)
        self.assertNotEqual(status, "error", msg=str(detail))
        self.assertIn(status, ("no_path", "timeout"),
                      msg="RRT found a valid path within 10 seconds")
    
    def test_rrt_beats_astar(self):
        
        np.random.seed(112)
        
        fname = os.path.join(os.path.dirname(__file__), "share", "rrt_beats_astar_map.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        problem = R2Problem(permissible_region)
        sampler = LatticeSampler(problem.extents)

        num_vertices = 100
        connection_radius = 3.0
        rm = Roadmap(problem, sampler, num_vertices, connection_radius)
        
        start = np.array([1, 1])
        goal = np.array([19, 19])
        start_id = rm.add_node(start, is_start=True)
        goal_id = rm.add_node(goal, is_start=False)

        with self.assertRaises(nx.NetworkXNoPath, msg="A* found a path"):
            astar = search.ASTARPlanner(rm)
            _ = astar.Plan(start_id, goal_id)
        
        status, detail = run_rrt(permissible_region, start, goal)
        self.assertEqual(status, "path",
                         msg="RRT did not return a valid path within 10 seconds: {} {}"
                         .format(status, detail))
        

if __name__ == "__main__":
    rosunit.unitrun("planning", "test_tradeoff", TestTradeoff)
