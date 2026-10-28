PKG = "hw3_pkg"
NAME = "pf_test"

import sys, unittest, time

import numpy as np
import math

from collections import Counter

import rospy, rostest

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

from hw3_pkg.particle_filter import particle_filter


class TestPF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pf = particle_filter(np.array([]), np.array([]))

        # create a publisher to send fake data
        cls.joint_states_publisher = rospy.Publisher("scan", LaserScan, queue_size=10)

        rospy.sleep(0.5)  # wait for publishers to init

    # this is called before each test fixture
    def setUp(self):
        self.pf = particle_filter(np.array([]), np.array([]))

    def __init__(self, *args):
        super(TestPF, self).__init__(*args)

    def test_sensor_model_single_val(self):
        prob = self.pf.sensor_model(np.array([1.0]), np.array([1.0]), 0.1)
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Incorrect unweighted probability"
        )

        prob = self.pf.sensor_model(np.array([1.0]), np.array([1.3]), 0.1)
        self.assertAlmostEqual(
            prob, 0.04431848411938, msg="Incorrect unweighted probability"
        )

    def test_sensor_model_ignores_nan(self):
        prob = self.pf.sensor_model(
            np.array([1.0, math.nan]), np.array([1.0, 2.0]), 0.1
        )
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Does not ignore NaN values in measurement"
        )

        prob = self.pf.sensor_model(
            np.array([1.0, 1.1]), np.array([1.0, math.nan]), 0.1
        )
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Does not ignore NaN values in ground_truth"
        )
    
    def test_sensor_model_ignores_inf(self):
        # Only the first pair is finite in both arrays. Invalid entries in
        # either array must be ignored, including both signs of infinity.
        prob = self.pf.sensor_model(
            np.array([1.0, np.inf, -np.inf, 2.0, 3.0, np.nan, 4.0]),
            np.array([1.0, 2.0, 3.0, np.inf, -np.inf, 4.0, np.nan]),
            0.1,
        )
        self.assertAlmostEqual(prob, 3.989422804014327)

    def test_sensor_model_many(self):
        prob = self.pf.sensor_model(
            np.array([1.0, 1.1, 1.2, 1.3]), np.array([1.0, 0.9, 0.8, 0.7]), 0.1
        )
        self.assertAlmostEqual(
            prob, 1.751438007529713e-10, msg="Does not ignore NaN values in measurement"
        )

    def test_resampler_inplace(self):
        n_particles = 16
        particles = np.arange(3 * n_particles, dtype=float).reshape(n_particles, 3)
        weights = np.zeros(n_particles)
        selected = n_particles // 2
        weights[selected] = 1.0
        expected_particles = np.repeat(particles[[selected]], n_particles, axis=0)

        rs = particle_filter(particles, weights)
        rs.resample()

        self.assertTrue(
            np.shares_memory(rs.particles, particles),
            msg="Resampler must retain the caller's particle storage",
        )
        np.testing.assert_array_equal(
            particles, expected_particles,
            err_msg="Resampler should modify particles in-place",
        )
        self.assertTrue(
            np.shares_memory(rs.weights, weights),
            msg="Resampler must retain the caller's weight storage",
        )
        np.testing.assert_allclose(
            weights, np.full(n_particles, 1.0 / n_particles),
            err_msg="Resampler should modify weights in-place",
        )

    def test_resampler_is_fair(self):
        n_particles = 128

        # Systematic resampling assigns floor(N*w) or ceil(N*w) copies to each
        # particle for every random offset. Integer N*w fixes the count exactly.
        # These weights are binary fractions, so the bounds are exact in float.
        for count_pattern in ([0, 0, 1, 3], [0, 0.5, 1.5, 2]):
            expected_counts = np.tile(count_pattern, n_particles // 4)
            particles = np.arange(n_particles)[:, np.newaxis]
            weights = expected_counts / n_particles

            rs = particle_filter(particles, weights)
            rs.resample()

            self.assertEqual(rs.particles.shape, (n_particles, 1))
            histogram = Counter(rs.particles[:, 0])
            counts = np.array([histogram[i] for i in range(n_particles)])
            self.assertEqual(
                counts.sum(), n_particles,
                msg="Resampling must only select particles from the input",
            )
            self.assertTrue(
                np.all(
                    (counts == np.floor(expected_counts))
                    | (counts == np.ceil(expected_counts))
                ),
                msg="Each particle must receive floor(N*w) or ceil(N*w) copies; "
                    "zero-weight particles must never be sampled (pattern: {})".format(count_pattern),
            )

    def test_resampler_is_complete(self):
        n_particles = 128
        cases = [
            ("uniform", np.ones(n_particles, dtype=int)),
            ("alternating", np.tile([0, 2], n_particles // 2)),
        ]
        for selected in (0, n_particles // 2, n_particles - 1):
            expected_counts = np.zeros(n_particles, dtype=int)
            expected_counts[selected] = n_particles
            cases.append(("all mass at {}".format(selected), expected_counts))

        for name, expected_counts in cases:
            particles = np.arange(3 * n_particles, dtype=float).reshape(
                n_particles, 3
            )
            weights = expected_counts / n_particles
            expected_histogram = Counter({
                tuple(row): int(count)
                for row, count in zip(particles, expected_counts) if count
            })

            rs = particle_filter(particles, weights)
            rs.resample()

            self.assertEqual(rs.particles.shape, (n_particles, 3))
            self.assertEqual(
                Counter(map(tuple, rs.particles)), expected_histogram,
                msg="Resampling must fill every output row with an intact "
                    "input particle and the required number of copies ({})".format(name),
            )

    def test_resampler_resets_weights(self):
        n_particles = 100  # number of particles

        # Start with deterministic, nonuniform weights so a no-op cannot pass.
        particles = np.arange(n_particles)
        particles = particles[..., np.newaxis]
        weights = np.arange(1, n_particles + 1, dtype=float)
        weights /= weights.sum()

        rs = particle_filter(particles, weights)
        rs.resample()

        np.testing.assert_allclose(
            weights,
            np.full(n_particles, 1.0 / n_particles),
            err_msg="The resampler should set the weights to be uniform",
        )


if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, TestPF, sys.argv)
