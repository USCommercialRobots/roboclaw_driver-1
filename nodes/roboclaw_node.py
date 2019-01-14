#!/usr/bin/env python

from __future__ import print_function
import traceback

from math import pi, cos, sin, copysign
import rospy
import diagnostic_updater
import diagnostic_msgs

from roboclaw_driver.msg import Stats, SpeedCommand
from roboclaw_driver import RoboclawControl, Roboclaw, RoboclawStub


DEFAULT_DEV_NAME = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw"
DEFAULT_LOOP_HZ = 100
DEFAULT_ADDRESS = 0x80
DEFAULT_DEADMAN_SEC = 3
DEFAULT_STATS_TOPIC = "~stats"
DEFAULT_SPEED_CMD_TOPIC = "~speed_command"



class RoboclawNode:
    def __init__(self, node_name):
        """
        Parameters:
            :param str node_name: ROS node name
            :param RoboclawControl rbc_ctl: RoboclawControl object that controls the hardware
            :param int loop_rate: Integer rate in Hz of the main loop
        """
        self._node_name = node_name
        self._rbc_ctl = None  # Set by the connect() method below

        # Records the time of the last velocity command
        # Initialize this now so we don't have to check for None values later
        self._last_cmd_time = rospy.get_rostime()

        # Set up the Publishers
        self._stats_pub = rospy.Publisher(
            rospy.get_param("~stats_topic", DEFAULT_STATS_TOPIC),
            Stats,
            queue_size=1
        )

        # Set up the Diagnostic Updater
        self._diag_updater = diagnostic_updater.Updater()
        self._diag_updater.setHardwareID(node_name)
        self._diag_updater.add("Read Diagnostics", self._publish_diagnostics)

        # Set up the SpeedCommand Subscriber
        rospy.Subscriber(
            rospy.get_param("~speed_cmd_topic", DEFAULT_SPEED_CMD_TOPIC),
            SpeedCommand,
            self._speed_cmd_callback
        )

        # For logdebug
        self.prev_m1_val = 0
        self.prev_m2_val = 0

    @property
    def roboclaw_control(self):
        return self._rbc_ctl

    def connect(self, dev_name, baud_rate, address, test_mode=False):
        """Connects the node to the Roboclaw controller, or the test stub
        Parameters:
            :param str dev_name: Serial device name (e.g. /dev/ttyACM0)
            :param int baud_rate: Serial baud rate (e.g. 115200)
            :param int address: Serial address (default 0x80)
            :param bool test_mode: True if connecting to the controller test stub
        """
        rospy.loginfo("Connecting to roboclaw")
        if not test_mode:
            roboclaw = Roboclaw(dev_name, baud_rate)
        else:
            roboclaw = RoboclawStub(dev_name, baud_rate)
        self._rbc_ctl = RoboclawControl(roboclaw, address)

    def run(self, loop_hz=10, deadman_secs=1):
        """Runs the main loop of the node
        Parameters:
            :param int loop_hz: Loops per sec of main loop (default 10 hertz)
            :param int deadman_secs: Seconds that the Roboclaw will continue the last command
                before stopping if another command is not received
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(loop_hz)

        try:
            rospy.loginfo("Starting main loop")

            while not rospy.is_shutdown():

                # Read and publish encoder readings
                stats = self._rbc_ctl.read_stats()
                for error in stats.error_messages:
                    rospy.logwarn(error)
                self._publish_stats(stats)

                # Stop motors if running and no commands are being received
                if (stats.m1_enc_qpps != 0 or stats.m2_enc_qpps != 0):
                    if (rospy.get_rostime() - self._last_cmd_time).to_sec() > deadman_secs:
                        rospy.loginfo("Did not receive a command for over 1 sec: Stopping motors")
                        self._rbc_ctl.stop()

                # Publish diagnostics
                self._diag_updater.update()

                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _publish_stats(self, stats):
        """Publish stats to the <node_name>/stats topic

        Parameters:
            :param roboclaw_control.RoboclawStats stats: Stats from the Roboclaw controller
        """
        msg = Stats()
        msg.header.stamp = rospy.get_rostime()

        msg.m1_enc_val = stats.m1_enc_val
        msg.m1_enc_qpps = stats.m1_enc_qpps

        msg.m2_enc_val = stats.m2_enc_val
        msg.m2_enc_qpps = stats.m2_enc_qpps

        # rospy.logdebug("Encoder diffs M1:{}, M2:{}".format(
        #     stats.m1_enc_val - self.prev_m1_val,
        #     stats.m2_enc_val - self.prev_m2_val
        # ))
        self.prev_m1_val = stats.m1_enc_val
        self.prev_m2_val = stats.m2_enc_val

        self._stats_pub.publish(msg)

    def _publish_diagnostics(self, stat):
        """Function called by the diagnostic_updater to fetch and publish diagnostics
        from the Roboclaw controller

        Parameters:
        :param diagnostic_updater.DiagnosticStatusWrapper stat:
            DiagnosticStatusWrapper provided by diagnostic_updater when called

        Returns: The updated DiagnosticStatusWrapper
        :rtype: diagnostic_updater.DiagnosticStatusWrapper
        """
        diag = self._rbc_ctl.read_diag()

        stat.add("Temperature 1 (C):", diag.temp1)
        stat.add("Temperature 2 (C):", diag.temp2)
        stat.add("Main Battery (V):", diag.main_battery_v)
        stat.add("Logic Battery (V):", diag.logic_battery_v)
        stat.add("Motor 1 current (Amps):", diag.m1_current)
        stat.add("Motor 2 current (Amps):", diag.m2_current)

        for msg in diag.error_messages:
            level = diagnostic_msgs.msg.DiagnosticStatus.WARN
            if "error" in msg:
                level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.summary(level, msg)

        return stat

    def _speed_cmd_callback(self, command):
        """
        Parameters:
            :param SpeedCommand command: The forward/turn command message
        """
        rospy.logdebug("Received SpeedCommand message")
        rospy.logdebug(
            "[M1 speed: {}] [M2 speed: {}] [Max Secs: {}]"
            .format(command.m1_qpps, command.m2_qpps, command.max_secs)
        )
        self._last_cmd_time = rospy.get_rostime()

        success = self._rbc_ctl.driveM1M2qpps(command.m1_qpps, command.m2_qpps, command.max_secs)
        if not success:
            rospy.logerr(
                "RoboclawControl SpeedAccelDistanceM1M2({}) failed".format(command.forward_pct)
            )

    def cmd_vel_callback(self, twist):
        last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        angular_z = twist.angular.z
        if abs(linear_x) > MAX_ABS_LINEAR_SPEED:
            linear_x = copysign(MAX_ABS_LINEAR_SPEED, linear_x)
        if abs(angular_z) > MAX_ABS_ANGULAR_SPEED:
            angular_z = copysign(MAX_ABS_ANGULAR_SPEED, angular_z)

        vr = linear_x - angular_z * BASE_WIDTH / 2.0  # m/s
        vl = linear_x + angular_z * BASE_WIDTH / 2.0

        vr_ticks = int(vr * TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * TICKS_PER_METER)

        # v_wheels= Wheels_speeds()
        # v_wheels.wheel1=vl
        # v_wheels.wheel2=vr
        # wheels_speeds_pub.publish(v_wheels)


        rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        self._last_cmd_time = rospy.get_rostime()

        success = self._rbc_ctl.driveM1M2qpps(vr_ticks, vl_ticks, 3)
        if not success:
            rospy.logerr(
                "RoboclawControl SpeedAccelDistanceM1M2 failed"
            )

        # try:
        #     #Replaced to implement watchdog
        #     #roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        #     #Replaced to implement acc
        #     #roboclaw.SpeedDistanceM1M2(self.address, vr_ticks, int(abs(vr_ticks*0.04)), vl_ticks, int(abs(vl_ticks*0.04)), 1)
        #     #rospy.logdebug(" Acc ticks %d" % (int(self.ACC_LIM * self.TICKS_PER_METER)))
        #     roboclaw.SpeedAccelDistanceM1(self.address, int(self.ACC_LIM * self.TICKS_PER_METER),vr_ticks, int(abs(vr_ticks*0.04)),1)
        #     roboclaw.SpeedAccelDistanceM2(self.address, int(self.ACC_LIM * self.TICKS_PER_METER),vl_ticks, int(abs(vl_ticks*0.04)),1)
        #     #Mixed command doesn't work
        #     #roboclaw.SpeedAccelDistanceM1M2(self.address, int(self.ACC_LIM * self.TICKS_PER_METER),vr_ticks, int(abs(vr_ticks*0.04)), vl_ticks, int(abs(vl_ticks*0.04)), 1)

        # except OSError as e:
        #     rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
        #     rospy.logdebug(e)

    def shutdown_node(self):
        """Performs Node shutdown tasks
        """
        rospy.loginfo("Shutting down...")


if __name__ == "__main__":

    # Setup the ROS node
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    print("node_name: {}".format(node_name))

    # Read the input parameters
    dev_name = rospy.get_param("~dev_name", DEFAULT_DEV_NAME)
    baud_rate = int(rospy.get_param("~baud", DEFAULT_BAUD_RATE))
    address = int(rospy.get_param("~address", DEFAULT_ADDRESS))
    loop_hz = int(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ))
    deadman_secs = int(rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SEC))
    test_mode = bool(rospy.get_param("~test_mode", False))
    MAX_ABS_LINEAR_SPEED = float(rospy.get_param("~max_abs_linear_speed", "1.0"))
    MAX_ABS_ANGULAR_SPEED = float(rospy.get_param("~max_abs_angular_speed", "1.0"))
    BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
    TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "4342.2"))

    rospy.logdebug("node_name: {}".format(node_name))
    rospy.logdebug("dev_name: {}".format(dev_name))
    rospy.logdebug("baud: {}".format(baud_rate))
    rospy.logdebug("address: {}".format(address))
    rospy.logdebug("loop_hz: {}".format(loop_hz))
    rospy.logdebug("deadman_secs: {}".format(deadman_secs))
    rospy.logdebug("test_mode: {}".format(test_mode))

    rospy.logdebug("max_abs_linear_speed %f", MAX_ABS_LINEAR_SPEED)
    rospy.logdebug("max_abs_angular_speed %f", MAX_ABS_ANGULAR_SPEED)
    rospy.logdebug("base_width %f", BASE_WIDTH)
    rospy.logdebug("ticks_per_meter %f", TICKS_PER_METER)

    node = RoboclawNode(node_name)
    rospy.on_shutdown(node.shutdown_node)

    try:
        # Initialize the Roboclaw controller
        node.connect(dev_name, baud_rate, address, test_mode)
        node.run(loop_hz=loop_hz, deadman_secs=DEFAULT_DEADMAN_SEC)

    except Exception as e:
        rospy.logfatal("Unhandled exeption...printing stack trace then shutting down node")
        rospy.logfatal(traceback.format_exc())

    # Shutdown and cleanup
    if node:
        node.shutdown_node()
    rospy.loginfo("Shutdown complete")
