#!/usr/bin/env python
from __future__ import print_function
import threading
import unittest

import rospy

from roboclaw_driver.msg import SpeedCommand, Stats

PKG = 'roboclaw_driver'
NAME = 'roboclaw_node_nodetest'


class TestRoboclawNode(unittest.TestCase):

    def __init__(self, *args):
        super(TestRoboclawNode, self).__init__(*args)

        self.lock = threading.RLock()
        self.stats = Stats()

        rospy.init_node("roboclaw_test", log_level=rospy.DEBUG)
        rospy.Subscriber("/roboclaw1/stats", Stats, self._stats_callback)
        self.pub_speed_cmd = rospy.Publisher(
            '/roboclaw1/speed_command', SpeedCommand, queue_size=1
        )
        rospy.sleep(1)  # Let subscribers connect

    def test_forward_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = 1000, 1000
        m1_dist, m2_dist = 2000, 2000
        qpps_delta = 0
        dist_delta = 1000

        cmd = self._create_speed_command(0, m1_qpps, m2_qpps, m1_dist, m2_dist)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + m1_dist, start_m2_dist + m2_dist, dist_delta
        )

    def test_reverse_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = -2000, -2000
        m1_dist, m2_dist = 4000, 4000
        qpps_delta = 0
        dist_delta = 2000

        cmd = self._create_speed_command(0, m1_qpps, m2_qpps, m1_dist, m2_dist)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist - m1_dist, start_m2_dist - m2_dist, dist_delta
        )

    def test_left_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = 1000, -1000
        m1_dist, m2_dist = 2000, 2000
        qpps_delta = 0
        dist_delta = 2000

        cmd = self._create_speed_command(0, m1_qpps, m2_qpps, m1_dist, m2_dist)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + m1_dist, start_m2_dist - m2_dist, dist_delta
        )

    def test_right_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = -1000, 1000
        m1_dist, m2_dist = 2000, 2000
        qpps_delta = 0
        dist_delta = 2000

        cmd = self._create_speed_command(0, m1_qpps, m2_qpps, m1_dist, m2_dist)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist - m1_dist, start_m2_dist + m2_dist, dist_delta
        )

    def _stats_callback(self, cmd):
        with self.lock:
            self.stats = cmd

    def _create_speed_command(self, accel, m1_speed, m2_speed, m1_dist, m2_dist):
        cmd = SpeedCommand()
        cmd.accel = accel
        cmd.m1_speed = m1_speed
        cmd.m2_speed = m2_speed
        cmd.m1_dist = m1_dist
        cmd.m2_dist = m2_dist
        return cmd

    def _check_stats(self, m1_qpps, m2_qpps, qpps_delta, m1_val, m2_val, val_delta):
        """Check actual stats values against expected values.
        Uses assertAlmostEqual comparison

        Parameters:
            :param int m1_qpps: Expected motor 1 QPPS value
            :param int m2_qpps: Expected motor 2 QPPS value
            :param int qpps_delta: Allowed difference between expected & actual QPPS
            :param int m1_val:  Expected motor 1 encoder value
            :param int m2_val:  Expected motor 2 encoder value
            :param int val_delta: Allowed difference between expected & actual encoder value
        """
        with self.lock:
            tests = [
                ("M1 QPPS", self.stats.m1_enc_qpps, m1_qpps, qpps_delta),
                ("M2 QPPS", self.stats.m2_enc_qpps, m2_qpps, qpps_delta),
                ("M1 encoder value", self.stats.m1_enc_val, m1_val, val_delta),
                ("M1 encoder value", self.stats.m2_enc_val, m2_val, val_delta),
            ]
            for label, actual_val, expected_val, delta in tests:
                self.assertAlmostEqual(
                    actual_val, expected_val, delta=delta,
                    msg="{} expected/delta: {}/{}, actual: {}".format(
                        label, expected_val, delta, actual_val
                    )
                )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestRoboclawNode)
