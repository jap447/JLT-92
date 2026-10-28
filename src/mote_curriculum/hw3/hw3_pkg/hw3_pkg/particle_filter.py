from __future__ import annotations

import numpy as np
import numpy.typing

import scipy.stats as stats


from hw3_pkg.map_helpers import gen_lidar_scan

from tf.transformations import quaternion_from_euler


class particle_filter:
    def __init__(
        self, particles: numpy.ndarray[float], weights: numpy.ndarray[np.float64]
    ):
        # Setup some particle variables
        self.rng = np.random.default_rng(42)
        self.particles = particles
        self.n_particles = particles.shape[0]
        self.weights = weights

    def sensor_model(
        self,
        measurement: np.ndarray[np.float64],
        ground_truth: np.ndarray[np.float64],
        std_dev: float,
    ) -> float:
        """
        INPUTS
         * measurement and ground_truth
               360 LiDAR readings, stored as a numpy array.
               Invalid readings are set to NaN.
         * std_dev: the standard deviation of the error in each reading

        OUTPUTS
         * the unnormalized probability of reading measurement when the actual distances are
           ground_truth

        HINTS:
         * If a reading from either measurement or ground_truth is NaN or inf, ignore it
           in both readings
         * Because we are assuming readings are IID, the joint probability of all
           readings is equal to their product.
         * We imported scipy.stats as stats at the top of this file.
           stats.norm.pdf may be of interest
        """

        # this will return 0 if no index contains finite readings (all indices are NaN/inf)
        if not np.any(np.isfinite(measurement) & np.isfinite(ground_truth)):
            return 0.0

        # QUESTION 1.1 BEGINS

        # QUESTION 1.1 ENDS

    def resample(self):
        """
        Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        Aim for an efficient O(M) implementation!

        Hint:
        * try to use numpy functions to keep your code efficient. There are many
          ways to approach this but cumsum and searchsorted will likely be helpful
        """

        # QUESTION 1.2 BEGINS

        # QUESTION 1.2 ENDS
        pass

    @staticmethod
    def wrap(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict(self, prev_odom, curr_odom, alpha=(0.5, 0.5, 0.5, 0.5), trans_jitter=0.011, rot_jitter=0.051):
        """
        prev_odom, curr_odom: (x, y, yaw) of the robot in the odom frame.
        alpha: noise params
        trans_jitter and rot_jitter: optional per-update noise floors
        """
        dx = curr_odom[0] - prev_odom[0]
        dy = curr_odom[1] - prev_odom[1]
        trans = np.hypot(dx, dy)

        rot1 = 0.0 if trans < 0.01 else self.wrap(np.arctan2(dy, dx) - prev_odom[2])
        rot2 = self.wrap(curr_odom[2] - prev_odom[2] - rot1)

        rot1_noise = min(abs(self.wrap(rot1)), abs(self.wrap(rot1 - np.pi)))
        rot2_noise = min(abs(self.wrap(rot2)), abs(self.wrap(rot2 - np.pi)))
        a1, a2, a3, a4 = alpha
        n = self.n_particles
        rot1_hat = rot1 - self.rng.normal(
            0, np.sqrt(a1 * rot1_noise**2 + a2 * trans**2 + rot_jitter**2), n)
        trans_hat = trans - self.rng.normal(
            0, np.sqrt(a3 * trans**2 + a4 * (rot1_noise**2 + rot2_noise**2) + trans_jitter**2), n)
        rot2_hat = rot2 - self.rng.normal(
            0, np.sqrt(a1 * rot2_noise**2 + a2 * trans**2 + rot_jitter**2), n)

        theta = self.particles[:, 2]
        self.particles[:, 0] += trans_hat * np.cos(theta + rot1_hat)
        self.particles[:, 1] += trans_hat * np.sin(theta + rot1_hat)
        self.particles[:, 2] = (theta + rot1_hat + rot2_hat) % (2 * np.pi)

    def update(self):
        weights = self.weights / self.weights.sum()
        mean = np.average(self.particles, weights=weights, axis=0)
        theta = self.particles[:, 2]
        mean[2] = np.arctan2(np.sum(weights * np.sin(theta)),
                             np.sum(weights * np.cos(theta)))
        delta = self.particles - mean
        delta[:, 2] = self.wrap(delta[:, 2])
        cov = (delta * weights[:, None]).T @ delta
        return mean, cov
