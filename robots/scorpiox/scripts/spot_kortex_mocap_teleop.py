#!/usr/bin/env python3
"""Mocap teleoperation for the Spot base and Kortex arm.

Phase one maps the torso's relative planar motion to a Spot pose target.  A
bounded proportional controller converts the target error to ``cmd_vel``. Arm
mode holds Spot still and maps relative hand translation to a MoveIt pose goal.
"""

import copy
import math
import sys
import threading
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


BASE_MODE = "base"
ARM_MODE = "arm"
VALID_MODES = (BASE_MODE, ARM_MODE)


class PlanarPose:
    __slots__ = ("x", "y", "yaw")

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class Position3D:
    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion):
    """Extract yaw from a geometry_msgs quaternion."""
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm < 1.0e-9:
        raise ValueError("received a zero-length quaternion")

    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def planar_pose(pose):
    result = PlanarPose(
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(pose.orientation),
    )
    if not all(math.isfinite(value) for value in (result.x, result.y, result.yaw)):
        raise ValueError("received a non-finite planar pose")
    return result


def position_3d(pose):
    result = Position3D(pose.position.x, pose.position.y, pose.position.z)
    if not all(math.isfinite(value) for value in (result.x, result.y, result.z)):
        raise ValueError("received a non-finite position")
    return result


def bounded_proportional(error, gain, minimum, maximum):
    """Return a positive P-control magnitude bounded by nonzero limits."""
    proportional = gain * abs(error)
    return min(max(proportional, minimum), maximum)


class SpotKortexMocapTeleop:
    def __init__(self):
        self.torso_topic = rospy.get_param("~torso_topic", "/torso/mocap/pose")
        self.hand_topic = rospy.get_param("~hand_topic", "/hand/mocap/pose")
        self.ground_truth_topic = rospy.get_param(
            "~ground_truth_topic", "/odom/ground_truth"
        )
        self.mode_topic = rospy.get_param(
            "~mode_topic", "/spot_kortex/control_mode"
        )
        self.mocap_tracking_topic = rospy.get_param(
            "~mocap_tracking_topic", "/spot_kortex/mocap_tracking_enabled"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.linear_kp = rospy.get_param("~linear_kp", 1.0)
        self.yaw_kp = rospy.get_param("~yaw_kp", 1.5)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.10)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.40)
        self.max_lateral_speed = rospy.get_param("~max_lateral_speed", 0.20)
        self.min_yaw_speed = rospy.get_param("~min_yaw_speed", 0.08)
        self.max_yaw_speed = rospy.get_param("~max_yaw_speed", 0.35)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.03)
        self.yaw_tolerance = rospy.get_param(
            "~yaw_tolerance", math.radians(2.0)
        )
        self.mocap_timeout = rospy.get_param("~mocap_timeout", 0.5)
        self.ground_truth_timeout = rospy.get_param(
            "~ground_truth_timeout", 0.5
        )
        self.tracking_command_timeout = rospy.get_param(
            "~tracking_command_timeout", 0.5
        )

        self.move_group_name = rospy.get_param("~move_group_name", "arm")
        self.robot_description = rospy.get_param(
            "~robot_description", "robot_description"
        )
        self.end_effector_link = rospy.get_param("~end_effector_link", "")
        self.moveit_wait_timeout = rospy.get_param("~moveit_wait_timeout", 10.0)
        self.moveit_planning_time = rospy.get_param("~moveit_planning_time", 5.0)
        self.moveit_planning_attempts = rospy.get_param(
            "~moveit_planning_attempts", 5
        )
        self.arm_velocity_scaling = rospy.get_param(
            "~arm_velocity_scaling", 0.20
        )
        self.arm_acceleration_scaling = rospy.get_param(
            "~arm_acceleration_scaling", 0.20
        )
        self.arm_position_tolerance = rospy.get_param(
            "~arm_position_tolerance", 0.005
        )
        self.arm_orientation_tolerance = rospy.get_param(
            "~arm_orientation_tolerance", 0.02
        )
        self.arm_translation_threshold = rospy.get_param(
            "~arm_translation_threshold", 0.015
        )
        self.arm_command_period = rospy.get_param("~arm_command_period", 0.50)
        self.arm_failure_retry_period = rospy.get_param(
            "~arm_failure_retry_period", 2.0
        )
        self.max_hand_displacement = rospy.get_param(
            "~max_hand_displacement", 0.50
        )
        self.move_group_cancel_topic = rospy.get_param(
            "~move_group_cancel_topic", "/move_group/cancel"
        )
        self.execute_trajectory_cancel_topic = rospy.get_param(
            "~execute_trajectory_cancel_topic", "/execute_trajectory/cancel"
        )
        self.arm_controller_cancel_topic = rospy.get_param(
            "~arm_controller_cancel_topic",
            "/arm_gen3_joint_trajectory_controller/follow_joint_trajectory/cancel",
        )
        self._validate_parameters()

        self._lock = threading.RLock()
        self.mode = BASE_MODE
        self.tracking_requested = False
        self.tracking_received_at = None
        self.tracking_active = False
        self.torso_pose = None
        self.hand_pose = None
        self.base_pose = None
        self.torso_received_at = None
        self.hand_received_at = None
        self.base_received_at = None
        self.torso_reference = None
        self.base_reference = None
        self.reference_pending = True
        self.arm_reference_pending = True
        self.hand_reference = None
        self.arm_torso_yaw_reference = None
        self.end_effector_reference = None
        self.last_arm_target = None
        self.last_arm_attempt_at = 0.0
        self.arm_inputs_valid = False
        self.moveit_commander = None
        self.arm_group = None
        self.arm_planning_frame = None
        self._shutdown_event = threading.Event()
        self._arm_wakeup = threading.Event()

        self.cmd_vel_publisher = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        self.move_group_cancel_publisher = rospy.Publisher(
            self.move_group_cancel_topic, GoalID, queue_size=1
        )
        self.execute_trajectory_cancel_publisher = rospy.Publisher(
            self.execute_trajectory_cancel_topic, GoalID, queue_size=1
        )
        self.arm_controller_cancel_publisher = rospy.Publisher(
            self.arm_controller_cancel_topic, GoalID, queue_size=1
        )
        self.torso_subscriber = rospy.Subscriber(
            self.torso_topic,
            PoseStamped,
            self._torso_callback,
            queue_size=1,
        )
        self.hand_subscriber = rospy.Subscriber(
            self.hand_topic,
            PoseStamped,
            self._hand_callback,
            queue_size=1,
        )
        self.ground_truth_subscriber = rospy.Subscriber(
            self.ground_truth_topic,
            Odometry,
            self._ground_truth_callback,
            queue_size=1,
        )
        self.mode_subscriber = rospy.Subscriber(
            self.mode_topic,
            String,
            self._mode_callback,
            queue_size=1,
        )
        self.mocap_tracking_subscriber = rospy.Subscriber(
            self.mocap_tracking_topic,
            Bool,
            self._mocap_tracking_callback,
            queue_size=1,
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._control_callback
        )
        self.arm_worker = threading.Thread(
            target=self._arm_worker_loop,
            name="spot_kortex_moveit_worker",
            daemon=True,
        )
        self.arm_worker.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Mocap teleop ready with tracking DISABLED in BASE mode: "
            "torso=%s, hand=%s, odom=%s, mode=%s, tracking=%s",
            self.torso_topic,
            self.hand_topic,
            self.ground_truth_topic,
            self.mode_topic,
            self.mocap_tracking_topic,
        )

    def _validate_parameters(self):
        positive = {
            "control_rate": self.control_rate,
            "linear_kp": self.linear_kp,
            "yaw_kp": self.yaw_kp,
            "max_linear_speed": self.max_linear_speed,
            "max_lateral_speed": self.max_lateral_speed,
            "max_yaw_speed": self.max_yaw_speed,
            "position_tolerance": self.position_tolerance,
            "yaw_tolerance": self.yaw_tolerance,
            "mocap_timeout": self.mocap_timeout,
            "ground_truth_timeout": self.ground_truth_timeout,
            "tracking_command_timeout": self.tracking_command_timeout,
            "moveit_wait_timeout": self.moveit_wait_timeout,
            "moveit_planning_time": self.moveit_planning_time,
            "arm_position_tolerance": self.arm_position_tolerance,
            "arm_orientation_tolerance": self.arm_orientation_tolerance,
            "arm_translation_threshold": self.arm_translation_threshold,
            "arm_command_period": self.arm_command_period,
            "arm_failure_retry_period": self.arm_failure_retry_period,
            "max_hand_displacement": self.max_hand_displacement,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError("{} must be greater than zero".format(name))

        if self.min_linear_speed < 0.0:
            raise ValueError("min_linear_speed cannot be negative")
        if self.min_yaw_speed < 0.0:
            raise ValueError("min_yaw_speed cannot be negative")
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError("min_linear_speed cannot exceed max_linear_speed")
        if self.min_yaw_speed > self.max_yaw_speed:
            raise ValueError("min_yaw_speed cannot exceed max_yaw_speed")
        if self.moveit_planning_attempts < 1:
            raise ValueError("moveit_planning_attempts must be at least one")
        for name, value in (
            ("arm_velocity_scaling", self.arm_velocity_scaling),
            ("arm_acceleration_scaling", self.arm_acceleration_scaling),
        ):
            if value <= 0.0 or value > 1.0:
                raise ValueError("{} must be in (0, 1]".format(name))

    def _torso_callback(self, message):
        try:
            pose = planar_pose(message.pose)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Ignoring torso pose: %s", error)
            return

        with self._lock:
            self.torso_pose = pose
            self.torso_received_at = rospy.Time.now()

    def _hand_callback(self, message):
        try:
            pose = position_3d(message.pose)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Ignoring hand pose: %s", error)
            return

        with self._lock:
            self.hand_pose = pose
            self.hand_received_at = rospy.Time.now()
        self._arm_wakeup.set()

    def _ground_truth_callback(self, message):
        try:
            pose = planar_pose(message.pose.pose)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Ignoring ground-truth pose: %s", error)
            return

        with self._lock:
            self.base_pose = pose
            self.base_received_at = rospy.Time.now()

    def _mode_callback(self, message):
        requested_mode = message.data.strip().lower()
        if requested_mode not in VALID_MODES:
            rospy.logwarn(
                "Ignoring unknown mocap control mode '%s'; expected base or arm",
                message.data,
            )
            return

        with self._lock:
            if requested_mode == self.mode:
                return
            self.mode = requested_mode
            self._reset_references_locked()

        self.stop()
        if requested_mode == BASE_MODE:
            self.cancel_arm_motion()
        self._arm_wakeup.set()
        rospy.loginfo("Mocap control mode changed to %s", requested_mode.upper())

    def _mocap_tracking_callback(self, message):
        with self._lock:
            tracking_requested = bool(message.data)
            disable_transition = self.tracking_requested and not tracking_requested
            self.tracking_requested = tracking_requested
            self.tracking_received_at = rospy.Time.now()

        if disable_transition:
            self.stop()
            self.cancel_arm_motion()
        self._arm_wakeup.set()

    def _reset_references_locked(self):
        self.reference_pending = True
        self.torso_reference = None
        self.base_reference = None
        self.arm_reference_pending = True
        self.hand_reference = None
        self.arm_torso_yaw_reference = None
        self.end_effector_reference = None
        self.last_arm_target = None
        self.last_arm_attempt_at = 0.0
        self.arm_inputs_valid = False

    @staticmethod
    def _is_stale(received_at, timeout, now):
        if received_at is None:
            return True
        age = (now - received_at).to_sec()
        return age < 0.0 or age > timeout

    def _capture_reference(self):
        self.torso_reference = PlanarPose(
            self.torso_pose.x, self.torso_pose.y, self.torso_pose.yaw
        )
        self.base_reference = PlanarPose(
            self.base_pose.x, self.base_pose.y, self.base_pose.yaw
        )
        self.reference_pending = False
        rospy.loginfo(
            "Captured BASE references: torso=(%.3f, %.3f, %.1f deg), "
            "Spot=(%.3f, %.3f, %.1f deg)",
            self.torso_reference.x,
            self.torso_reference.y,
            math.degrees(self.torso_reference.yaw),
            self.base_reference.x,
            self.base_reference.y,
            math.degrees(self.base_reference.yaw),
        )

    def _base_target(self):
        torso_dx = self.torso_pose.x - self.torso_reference.x
        torso_dy = self.torso_pose.y - self.torso_reference.y

        # Align the mocap torso's initial heading with Spot's initial heading.
        alignment_yaw = self.base_reference.yaw - self.torso_reference.yaw
        cosine = math.cos(alignment_yaw)
        sine = math.sin(alignment_yaw)
        base_dx = cosine * torso_dx - sine * torso_dy
        base_dy = sine * torso_dx + cosine * torso_dy

        torso_yaw_delta = wrap_angle(
            self.torso_pose.yaw - self.torso_reference.yaw
        )
        return PlanarPose(
            self.base_reference.x + base_dx,
            self.base_reference.y + base_dy,
            wrap_angle(self.base_reference.yaw + torso_yaw_delta),
        )

    def _base_command(self, target):
        error_world_x = target.x - self.base_pose.x
        error_world_y = target.y - self.base_pose.y
        position_error = math.hypot(error_world_x, error_world_y)
        yaw_error = wrap_angle(target.yaw - self.base_pose.yaw)

        command = Twist()
        if position_error > self.position_tolerance:
            linear_speed = bounded_proportional(
                position_error,
                self.linear_kp,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            velocity_world_x = linear_speed * error_world_x / position_error
            velocity_world_y = linear_speed * error_world_y / position_error

            # CHAMP consumes cmd_vel in Spot's body frame.
            cosine = math.cos(self.base_pose.yaw)
            sine = math.sin(self.base_pose.yaw)
            command.linear.x = cosine * velocity_world_x + sine * velocity_world_y
            command.linear.y = -sine * velocity_world_x + cosine * velocity_world_y
            command.linear.x = max(
                -self.max_linear_speed,
                min(self.max_linear_speed, command.linear.x),
            )
            command.linear.y = max(
                -self.max_lateral_speed,
                min(self.max_lateral_speed, command.linear.y),
            )

        if abs(yaw_error) > self.yaw_tolerance:
            yaw_speed = bounded_proportional(
                yaw_error,
                self.yaw_kp,
                self.min_yaw_speed,
                self.max_yaw_speed,
            )
            command.angular.z = math.copysign(yaw_speed, yaw_error)

        return command, position_error, yaw_error

    def _control_callback(self, _event):
        with self._lock:
            now = rospy.Time.now()
            tracking_active = self.tracking_requested and not self._is_stale(
                self.tracking_received_at,
                self.tracking_command_timeout,
                now,
            )
            tracking_transition = tracking_active != self.tracking_active
            if tracking_transition:
                self.tracking_active = tracking_active
                self._reset_references_locked()
                self.stop()
                if not tracking_active:
                    self.cancel_arm_motion()
                self._arm_wakeup.set()
                rospy.loginfo(
                    "Mocap tracking %s; references will be captured on fresh data",
                    "ENABLED" if tracking_active else "DISABLED",
                )

            if not self.tracking_active:
                return

            if self.mode == ARM_MODE:
                self.cmd_vel_publisher.publish(Twist())
                hand_stale = self._is_stale(
                    self.hand_received_at, self.mocap_timeout, now
                )
                torso_stale = self.arm_reference_pending and self._is_stale(
                    self.torso_received_at, self.mocap_timeout, now
                )
                inputs_valid = not hand_stale and not torso_stale
                cancel_motion = self.arm_inputs_valid and not inputs_valid
                self.arm_inputs_valid = inputs_valid

                if inputs_valid:
                    self._arm_wakeup.set()
                    rospy.loginfo_throttle(
                        5.0,
                        "ARM mode: Spot is held still and hand targets are enabled",
                    )
                else:
                    missing = []
                    if hand_stale:
                        missing.append(self.hand_topic)
                    if torso_stale:
                        missing.append(self.torso_topic + " (reference only)")
                    rospy.logwarn_throttle(
                        2.0,
                        "ARM mode stopped: waiting for fresh %s",
                        " and ".join(missing),
                    )
                if cancel_motion:
                    self.cancel_arm_motion()
                return

            torso_stale = self._is_stale(
                self.torso_received_at, self.mocap_timeout, now
            )
            base_stale = self._is_stale(
                self.base_received_at, self.ground_truth_timeout, now
            )
            if torso_stale or base_stale:
                self.reference_pending = True
                self.cmd_vel_publisher.publish(Twist())
                missing = []
                if torso_stale:
                    missing.append(self.torso_topic)
                if base_stale:
                    missing.append(self.ground_truth_topic)
                rospy.logwarn_throttle(
                    2.0,
                    "BASE mode stopped: waiting for fresh %s",
                    " and ".join(missing),
                )
                return

            if self.reference_pending:
                self._capture_reference()
                self.cmd_vel_publisher.publish(Twist())
                return

            target = self._base_target()
            command, position_error, yaw_error = self._base_command(target)
            self.cmd_vel_publisher.publish(command)
            rospy.loginfo_throttle(
                1.0,
                "BASE target=(%.3f, %.3f, %.1f deg), error=(%.3f m, %.1f deg), "
                "cmd=(%.3f, %.3f, %.3f)",
                target.x,
                target.y,
                math.degrees(target.yaw),
                position_error,
                math.degrees(yaw_error),
                command.linear.x,
                command.linear.y,
                command.angular.z,
            )

    def _initialize_moveit(self):
        import moveit_commander

        moveit_commander.roscpp_initialize(sys.argv)
        arm_group = moveit_commander.MoveGroupCommander(
            self.move_group_name,
            robot_description=self.robot_description,
            ns="",
            wait_for_servers=self.moveit_wait_timeout,
        )
        arm_group.set_planning_time(self.moveit_planning_time)
        arm_group.set_num_planning_attempts(self.moveit_planning_attempts)
        arm_group.set_max_velocity_scaling_factor(self.arm_velocity_scaling)
        arm_group.set_max_acceleration_scaling_factor(
            self.arm_acceleration_scaling
        )
        arm_group.set_goal_position_tolerance(self.arm_position_tolerance)
        arm_group.set_goal_orientation_tolerance(self.arm_orientation_tolerance)

        planning_frame = arm_group.get_planning_frame()
        end_effector_link = self.end_effector_link or arm_group.get_end_effector_link()
        if not planning_frame:
            raise RuntimeError("MoveIt returned an empty planning frame")
        if not end_effector_link:
            raise RuntimeError("MoveIt returned an empty end-effector link")
        arm_group.set_pose_reference_frame(planning_frame)

        self.moveit_commander = moveit_commander
        self.arm_group = arm_group
        self.arm_planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        rospy.loginfo(
            "MoveIt hand control ready: group=%s, frame=%s, end_effector=%s",
            self.move_group_name,
            self.arm_planning_frame,
            self.end_effector_link,
        )

    def _arm_data_ready(self):
        with self._lock:
            if (
                not self.tracking_requested
                or not self.tracking_active
                or self.mode != ARM_MODE
            ):
                return False
            now = rospy.Time.now()
            if self._is_stale(self.hand_received_at, self.mocap_timeout, now):
                return False
            if self.arm_reference_pending and self._is_stale(
                self.torso_received_at, self.mocap_timeout, now
            ):
                return False
            return True

    def _capture_arm_reference(self):
        with self._lock:
            if not self._arm_data_ready():
                return False
            hand_reference = Position3D(
                self.hand_pose.x, self.hand_pose.y, self.hand_pose.z
            )
            torso_yaw_reference = self.torso_pose.yaw

        current_pose = self.arm_group.get_current_pose(
            self.end_effector_link
        ).pose

        with self._lock:
            if (
                not self.tracking_requested
                or not self.tracking_active
                or self.mode != ARM_MODE
            ):
                return False
            self.hand_reference = hand_reference
            self.arm_torso_yaw_reference = torso_yaw_reference
            self.end_effector_reference = copy.deepcopy(current_pose)
            self.last_arm_target = copy.deepcopy(current_pose)
            self.arm_reference_pending = False

        rospy.loginfo(
            "Captured ARM references: hand=(%.3f, %.3f, %.3f), "
            "%s=(%.3f, %.3f, %.3f); end-effector orientation will be held",
            hand_reference.x,
            hand_reference.y,
            hand_reference.z,
            self.end_effector_link,
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
        )
        return True

    def _arm_target(self):
        with self._lock:
            hand = Position3D(self.hand_pose.x, self.hand_pose.y, self.hand_pose.z)
            hand_reference = Position3D(
                self.hand_reference.x,
                self.hand_reference.y,
                self.hand_reference.z,
            )
            torso_yaw_reference = self.arm_torso_yaw_reference
            end_effector_reference = copy.deepcopy(self.end_effector_reference)
            last_arm_target = copy.deepcopy(self.last_arm_target)

        hand_dx = hand.x - hand_reference.x
        hand_dy = hand.y - hand_reference.y
        hand_dz = hand.z - hand_reference.z
        hand_displacement = math.sqrt(
            hand_dx * hand_dx + hand_dy * hand_dy + hand_dz * hand_dz
        )

        # Express mocap-world translation in Spot's body frame at ARM entry.
        cosine = math.cos(torso_yaw_reference)
        sine = math.sin(torso_yaw_reference)
        body_dx = cosine * hand_dx + sine * hand_dy
        body_dy = -sine * hand_dx + cosine * hand_dy

        target = Pose()
        target.position.x = end_effector_reference.position.x + body_dx
        target.position.y = end_effector_reference.position.y + body_dy
        target.position.z = end_effector_reference.position.z + hand_dz
        target.orientation = copy.deepcopy(end_effector_reference.orientation)

        target_change = math.sqrt(
            (target.position.x - last_arm_target.position.x) ** 2
            + (target.position.y - last_arm_target.position.y) ** 2
            + (target.position.z - last_arm_target.position.z) ** 2
        )
        return target, hand_displacement, target_change

    @staticmethod
    def _unpack_plan(plan_result):
        if isinstance(plan_result, tuple):
            return bool(plan_result[0]), plan_result[1]
        trajectory = plan_result
        success = bool(trajectory.joint_trajectory.points)
        return success, trajectory

    def _plan_and_execute_arm_target(self, target):
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_pose_target(target, self.end_effector_link)
        rospy.loginfo(
            "Planning hand target in %s: (%.3f, %.3f, %.3f)",
            self.arm_planning_frame,
            target.position.x,
            target.position.y,
            target.position.z,
        )

        try:
            plan_success, trajectory = self._unpack_plan(self.arm_group.plan())
            if not plan_success or not trajectory.joint_trajectory.points:
                rospy.logwarn("MoveIt could not plan the current hand target")
                return False

            with self._lock:
                execute_allowed = (
                    self.tracking_requested
                    and self.tracking_active
                    and self.mode == ARM_MODE
                    and self.arm_inputs_valid
                )
            if not execute_allowed:
                rospy.loginfo("Discarding planned hand target because ARM mode stopped")
                return False

            execute_success = self.arm_group.execute(trajectory, wait=True)
            self.arm_group.stop()
            if not execute_success:
                rospy.logwarn("MoveIt failed or cancelled hand-target execution")
                return False

            rospy.loginfo(
                "Reached hand target: (%.3f, %.3f, %.3f)",
                target.position.x,
                target.position.y,
                target.position.z,
            )
            return True
        finally:
            self.arm_group.clear_pose_targets()

    def _arm_worker_loop(self):
        while not self._shutdown_event.is_set() and not rospy.is_shutdown():
            self._arm_wakeup.wait(timeout=0.2)
            self._arm_wakeup.clear()
            if self._shutdown_event.is_set() or rospy.is_shutdown():
                return
            if not self._arm_data_ready():
                continue

            if self.arm_group is None:
                try:
                    self._initialize_moveit()
                except Exception as error:
                    rospy.logerr_throttle(
                        5.0, "MoveIt hand-control initialization failed: %s", error
                    )
                    self._shutdown_event.wait(self.arm_failure_retry_period)
                    self._arm_wakeup.set()
                    continue

            with self._lock:
                reference_pending = self.arm_reference_pending
            if reference_pending:
                try:
                    self._capture_arm_reference()
                except Exception as error:
                    rospy.logerr_throttle(
                        5.0, "Failed to capture ARM reference: %s", error
                    )
                continue

            target, hand_displacement, target_change = self._arm_target()
            if hand_displacement > self.max_hand_displacement:
                rospy.logwarn_throttle(
                    2.0,
                    "Ignoring hand displacement %.3f m above safety limit %.3f m",
                    hand_displacement,
                    self.max_hand_displacement,
                )
                continue
            if target_change < self.arm_translation_threshold:
                continue

            elapsed = time.monotonic() - self.last_arm_attempt_at
            if elapsed < self.arm_command_period:
                self._shutdown_event.wait(self.arm_command_period - elapsed)
                self._arm_wakeup.set()
                continue

            self.last_arm_attempt_at = time.monotonic()
            try:
                success = self._plan_and_execute_arm_target(target)
            except Exception as error:
                rospy.logerr("MoveIt hand target failed: %s", error)
                success = False

            if success:
                with self._lock:
                    self.last_arm_target = copy.deepcopy(target)
            else:
                self._shutdown_event.wait(self.arm_failure_retry_period)
            self._arm_wakeup.set()

    def cancel_arm_motion(self):
        cancel = GoalID()
        self.move_group_cancel_publisher.publish(cancel)
        self.execute_trajectory_cancel_publisher.publish(cancel)
        self.arm_controller_cancel_publisher.publish(cancel)

    def stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def shutdown(self):
        self._shutdown_event.set()
        self._arm_wakeup.set()
        self.cancel_arm_motion()
        self.stop()


def main():
    rospy.init_node("spot_kortex_mocap_teleop")
    try:
        SpotKortexMocapTeleop()
    except ValueError as error:
        rospy.logfatal("Invalid mocap teleop configuration: %s", error)
        raise SystemExit(1)
    rospy.spin()


if __name__ == "__main__":
    main()
