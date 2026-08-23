#!/usr/bin/env python3
"""Keyboard and joystick teleoperation for the Spot base of Spot + Kortex."""

import math
import os
import select
import sys
import termios
import tty

import rospy
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Joy


HELP = """
Spot + Kortex base teleoperation
--------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Hold Shift for holonomic strafing:
   U    I    O
   J    K    L
   M    <    >

q/z : increase/decrease linear and angular speed by 10%
w/x : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%
f/h : decrease/increase body roll
t/b : increase/decrease body pitch
r/y : increase/decrease body yaw
space or any other key : stop
Ctrl-C : quit
"""


VELOCITY_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "v": (0, 0, 1, 0),
    "n": (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}

POSE_BINDINGS = {
    "f": ("roll", -1.0),
    "h": ("roll", 1.0),
    "t": ("pitch", 1.0),
    "b": ("pitch", -1.0),
    "r": ("yaw", 1.0),
    "y": ("yaw", -1.0),
}


def quaternion_from_euler(roll, pitch, yaw):
    """Return a geometry-compatible quaternion tuple (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SpotKortexTeleop:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pose_lite_publisher = rospy.Publisher(
            "body_pose/raw", PoseLite, queue_size=1
        )
        self.pose_publisher = rospy.Publisher("body_pose", Pose, queue_size=1)

        self.joy_mode = rospy.get_param("~joy", False)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        self.key_timeout = rospy.get_param("~key_timeout", 0.1)
        self.pose_step = rospy.get_param("~pose_step", math.radians(1.0))
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0

        self.joy_subscriber = None
        if self.joy_mode:
            self.joy_subscriber = rospy.Subscriber(
                "joy", Joy, self.joy_callback, queue_size=1
            )

        rospy.on_shutdown(self.stop)

    def publish_velocity(self, x, y, z, yaw):
        command = Twist()
        command.linear.x = x * self.speed
        command.linear.y = y * self.speed
        command.linear.z = z * self.speed
        command.angular.z = yaw * self.turn
        self.velocity_publisher.publish(command)

    def publish_body_pose(self, body_pose_lite):
        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.x = body_pose_lite.x
        body_pose.position.y = body_pose_lite.y
        body_pose.position.z = body_pose_lite.z
        quaternion = quaternion_from_euler(
            body_pose_lite.roll,
            body_pose_lite.pitch,
            body_pose_lite.yaw,
        )
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]
        self.pose_publisher.publish(body_pose)

    def joy_callback(self, data):
        if len(data.axes) < 6 or len(data.buttons) < 6:
            rospy.logwarn_throttle(
                5.0,
                "Joy message needs at least 6 axes and 6 buttons; got %d axes and %d buttons",
                len(data.axes),
                len(data.buttons),
            )
            return

        self.publish_velocity(
            data.axes[1],
            data.buttons[4] * data.axes[0],
            0.0,
            (not data.buttons[4]) * data.axes[0],
        )

        body_pose = PoseLite()
        body_pose.roll = (not data.buttons[5]) * -data.axes[3] * 0.349066
        body_pose.pitch = data.axes[4] * 0.174533
        body_pose.yaw = data.buttons[5] * data.axes[3] * 0.436332
        if data.axes[5] < 0.0:
            body_pose.z = data.axes[5] * 0.5
        self.publish_body_pose(body_pose)

    def adjust_body_pose(self, axis, direction):
        limits = {
            "roll": math.radians(30.0),
            "pitch": math.radians(20.0),
            "yaw": math.radians(25.0),
        }
        attribute = "body_{}".format(axis)
        value = getattr(self, attribute) + direction * self.pose_step
        value = max(-limits[axis], min(limits[axis], value))
        setattr(self, attribute, value)

        body_pose = PoseLite()
        body_pose.roll = self.body_roll
        body_pose.pitch = self.body_pitch
        body_pose.yaw = self.body_yaw
        self.publish_body_pose(body_pose)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        try:
            readable, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
            return sys.stdin.read(1) if readable else ""
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def stop(self):
        self.velocity_publisher.publish(Twist())

    def run_keyboard(self):
        if not os.isatty(sys.stdin.fileno()):
            raise RuntimeError("Keyboard teleop requires an interactive terminal (TTY)")

        settings = termios.tcgetattr(sys.stdin)
        moving = False
        print(HELP)
        print(self.velocity_status())

        try:
            while not rospy.is_shutdown():
                key = self.get_key(settings)

                if key in VELOCITY_BINDINGS:
                    self.publish_velocity(*VELOCITY_BINDINGS[key])
                    moving = True
                elif key in POSE_BINDINGS:
                    axis, direction = POSE_BINDINGS[key]
                    self.adjust_body_pose(axis, direction)
                elif key in SPEED_BINDINGS:
                    speed_scale, turn_scale = SPEED_BINDINGS[key]
                    self.speed *= speed_scale
                    self.turn *= turn_scale
                    print(self.velocity_status())
                elif key == "\x03":
                    break
                elif moving:
                    self.stop()
                    moving = False
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def velocity_status(self):
        return "currently: speed {:.3f}, turn {:.3f}".format(self.speed, self.turn)

    def run(self):
        if self.joy_mode:
            rospy.loginfo("Spot + Kortex teleop is using the joy topic")
            rospy.spin()
        else:
            self.run_keyboard()


def main():
    rospy.init_node("spot_kortex_teleop")
    SpotKortexTeleop().run()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, RuntimeError) as error:
        rospy.logerr("Spot + Kortex teleop stopped: %s", error)
        raise SystemExit(1)
