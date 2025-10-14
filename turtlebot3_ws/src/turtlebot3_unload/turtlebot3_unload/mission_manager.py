#!/usr/bin/env python3
"""Mission manager for TurtleBot3 unload workflow."""

import enum
import math
import pathlib
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger

import tf2_ros

from ament_index_python.packages import get_package_share_directory

import yaml
from rclpy.exceptions import ParameterAlreadyDeclaredException


def build_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


@dataclass
class Destination:
    marker_id: int
    staging_pose: PoseStamped
    return_pose: PoseStamped


class MissionState(enum.Enum):
    IDLE = enum.auto()
    NAVIGATE = enum.auto()
    SEARCH = enum.auto()
    AUTOPARK = enum.auto()
    UNLOAD = enum.auto()
    RETURN = enum.auto()
    COMPLETE = enum.auto()


class MissionManager(Node):

    def __init__(self) -> None:
        super().__init__(
            'turtlebot3_unload_mission',
            automatically_declare_parameters_from_overrides=True
        )

        # Parameters
        self._declare_if_absent('destination_topic', '/unload/destination')
        self._declare_if_absent('status_topic', '/unload/status')
        self._declare_if_absent('destination_config', 'config/destinations.yaml')
        self._declare_if_absent('parking_status_topic', '/automatic_parking/status')
        self._declare_if_absent('parking_reset_service', '/automatic_parking/reset')
        self._declare_if_absent('target_marker_topic', '/target_marker_id')
        self._declare_if_absent('marker_search/rotation_speed', 0.3)
        self._declare_if_absent('marker_search/max_search_time', 45.0)
        self._declare_if_absent('dumpbox_pwm_topic', '/turtlebot3_dumpbox/servo/pwm')
        self._declare_if_absent('dumpbox_service', 'turtlebot3_dumpbox/servo/apply_default')
        self._declare_if_absent('unload/servo_pwm_open', 1000.0)
        self._declare_if_absent('unload/servo_pwm_close', 1500.0)
        self._declare_if_absent('unload/servo_hold_time', 3.0)

        config_path = self._resolve_config_path(
            self.get_parameter('destination_config').get_parameter_value().string_value)
        self._destinations, self._default_home = self._load_config(config_path)

        qos = QoSProfile(depth=10)
        self._status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._marker_pub = self.create_publisher(Int32, self.get_parameter('target_marker_topic').value, qos)
        self._servo_pub = self.create_publisher(Float32, self.get_parameter('dumpbox_pwm_topic').value, qos)

        self.create_subscription(
            Int32,
            self.get_parameter('destination_topic').value,
            self._on_destination,
            qos)

        self.create_subscription(
            String,
            self.get_parameter('parking_status_topic').value,
            self._on_parking_status,
            qos)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._dumpbox_client = self.create_client(Trigger, self.get_parameter('dumpbox_service').value)
        self._parking_reset_client = self.create_client(
            Trigger,
            self.get_parameter('parking_reset_service').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._state = MissionState.IDLE
        self._requested_id: Optional[int] = None
        self._active_destination: Optional[Destination] = None
        self._nav_goal_in_progress = False
        self._marker_search_start = None
        self._parking_complete = False
        self._unload_started = False
        self._unload_start_time = None

        self._state_timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'turtlebot3_unload mission manager ready (config: {config_path})'
        )

    def _declare_if_absent(self, name: str, default_value):
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            pass

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------
    def _resolve_config_path(self, config_value: str) -> pathlib.Path:
        path = pathlib.Path(config_value)
        if path.is_file():
            return path
        share_dir = pathlib.Path(get_package_share_directory('turtlebot3_unload'))
        resolved = share_dir / config_value
        if not resolved.is_file():
            raise FileNotFoundError(f'Destination config not found: {resolved}')
        return resolved

    def _load_config(self, path: pathlib.Path) -> (Dict[int, Destination], PoseStamped):
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        destinations: Dict[int, Destination] = {}
        for key, entry in data.get('destinations', {}).items():
            dest_id = int(key)
            dest = Destination(
                marker_id=int(entry['marker_id']),
                staging_pose=build_pose(
                    entry['staging_pose']['x'],
                    entry['staging_pose']['y'],
                    entry['staging_pose']['yaw']
                ),
                return_pose=build_pose(
                    entry['return_pose']['x'],
                    entry['return_pose']['y'],
                    entry['return_pose']['yaw']
                )
            )
            destinations[dest_id] = dest

        home = data.get('default_home', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        default_home = build_pose(home['x'], home['y'], home['yaw'])

        search_cfg = data.get('marker_search', {})
        overrides = []
        if 'rotation_speed' in search_cfg:
            overrides.append(Parameter(
                'marker_search/rotation_speed', Parameter.Type.DOUBLE, float(search_cfg['rotation_speed'])
            ))
        if 'max_search_time' in search_cfg:
            overrides.append(Parameter(
                'marker_search/max_search_time', Parameter.Type.DOUBLE, float(search_cfg['max_search_time'])
            ))
        if overrides:
            self.set_parameters(overrides)

        unload_cfg = data.get('unload', {})
        unload_overrides = []
        if 'servo_pwm_open' in unload_cfg:
            unload_overrides.append(Parameter('unload/servo_pwm_open', Parameter.Type.DOUBLE,
                                              float(unload_cfg['servo_pwm_open'])))
        if 'servo_pwm_close' in unload_cfg:
            unload_overrides.append(Parameter('unload/servo_pwm_close', Parameter.Type.DOUBLE,
                                              float(unload_cfg['servo_pwm_close'])))
        if 'servo_hold_time' in unload_cfg:
            unload_overrides.append(Parameter('unload/servo_hold_time', Parameter.Type.DOUBLE,
                                              float(unload_cfg['servo_hold_time'])))
        if unload_overrides:
            self.set_parameters(unload_overrides)

        return destinations, default_home

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_destination(self, msg: Int32) -> None:
        dest_id = msg.data
        if dest_id not in self._destinations:
            self.get_logger().warn(f'Unknown destination id: {dest_id}')
            return
        self.get_logger().info(f'Received destination request: {dest_id}')
        self._requested_id = dest_id
        if self._state == MissionState.IDLE:
            self._transition(MissionState.NAVIGATE)

    def _on_parking_status(self, msg: String) -> None:
        text = msg.data.lower()
        if 'complete' in text or 'finished' in text:
            self._parking_complete = True
        if 'reset' in text or 'wait' in text:
            self._parking_complete = False

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _transition(self, new_state: MissionState, detail: Optional[str] = None) -> None:
        detail_suffix = f' ({detail})' if detail else ''
        self.get_logger().info(
            f'State: {self._state.name} -> {new_state.name}{detail_suffix}'
        )
        self._state = new_state
        self._publish_status(detail)
        if new_state == MissionState.NAVIGATE:
            self._start_navigation()
        elif new_state == MissionState.SEARCH:
            self._begin_search()
        elif new_state == MissionState.AUTOPARK:
            self._start_autopark()
        elif new_state == MissionState.UNLOAD:
            self._begin_unload()
        elif new_state == MissionState.RETURN:
            self._start_return()
        elif new_state == MissionState.COMPLETE:
            self._finalize_mission()

    def _publish_status(self, detail: Optional[str] = None) -> None:
        marker = self._active_destination.marker_id if self._active_destination else -1
        msg = String()
        suffix = f', detail={detail}' if detail else ''
        msg.data = f'state={self._state.name}, marker={marker}{suffix}'
        self._status_pub.publish(msg)

    def _tick(self) -> None:
        if self._state == MissionState.SEARCH:
            self._handle_search_state()
        elif self._state == MissionState.AUTOPARK:
            self._handle_autopark_state()
        elif self._state == MissionState.UNLOAD:
            self._handle_unload_state()

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------
    def _start_navigation(self) -> None:
        if self._requested_id is None:
            self.get_logger().warn('No destination requested.')
            return
        self._active_destination = self._destinations[self._requested_id]
        pose = self._clone_pose(self._active_destination.staging_pose)
        self._send_nav_goal(pose)

    def _start_return(self) -> None:
        pose = self._clone_pose(
            self._active_destination.return_pose if self._active_destination else self._default_home)
        self._send_nav_goal(pose)

    def _send_nav_goal(self, pose: PoseStamped) -> None:
        pose.header.stamp = self.get_clock().now().to_msg()
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'Sending navigation goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        self._nav_goal_in_progress = True
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self._nav_goal_in_progress = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._nav_goal_in_progress = False
        result = future.result()
        if result.status == 4:
            if self._state == MissionState.NAVIGATE:
                self._transition(MissionState.SEARCH)
            elif self._state == MissionState.RETURN:
                self._transition(MissionState.COMPLETE)
        else:
            self.get_logger().warn(f'Navigation failed with status {result.status}')
            if self._state in (MissionState.NAVIGATE, MissionState.RETURN):
                self._transition(MissionState.COMPLETE, 'navigation_failed')

    def _on_nav_feedback(self, feedback) -> None:
        pose = feedback.feedback.current_pose.pose
        self.get_logger().debug(f'NAV feedback: ({pose.position.x:.2f}, {pose.position.y:.2f})')

    def _clone_pose(self, template: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = template.header.frame_id or 'map'
        pose.pose = template.pose
        return pose

    # ------------------------------------------------------------------
    # Search and parking
    # ------------------------------------------------------------------
    def _begin_search(self) -> None:
        self._marker_search_start = self.get_clock().now()
        self._parking_complete = False
        self._publish_marker_id()
        self.get_logger().info(
            f'Searching for marker {self._active_destination.marker_id}'
        )

    def _publish_marker_id(self) -> None:
        if self._active_destination:
            self._marker_pub.publish(Int32(data=self._active_destination.marker_id))

    def _handle_search_state(self) -> None:
        if not self._active_destination:
            return
        if self._marker_visible():
            self._cmd_vel_pub.publish(Twist())
            self._transition(MissionState.AUTOPARK)
            return

        if self._marker_search_start is None:
            self._marker_search_start = self.get_clock().now()

        elapsed = (self.get_clock().now() - self._marker_search_start).nanoseconds / 1e9
        timeout = self.get_parameter('marker_search/max_search_time').get_parameter_value().double_value
        if elapsed > timeout:
            self.get_logger().warn(f'Marker search timeout ({elapsed:.1f}s)')
            self._cmd_vel_pub.publish(Twist())
            self._request_parking_reset()
            self._transition(MissionState.RETURN, 'search_timeout')
            return

        twist = Twist()
        twist.angular.z = self.get_parameter('marker_search/rotation_speed').get_parameter_value().double_value
        self._cmd_vel_pub.publish(twist)

    def _marker_visible(self) -> bool:
        try:
            marker_frame = f'ar_marker_{self._active_destination.marker_id}'
            self._tf_buffer.lookup_transform('base_link', marker_frame, rclpy.time.Time())
            return True
        except Exception:
            return False

    def _start_autopark(self) -> None:
        self._parking_complete = False
        self._publish_marker_id()
        self.get_logger().info('Automatic parking engaged')

    def _handle_autopark_state(self) -> None:
        if self._parking_complete:
            self.get_logger().info('Parking complete, moving to unload')
            self._cmd_vel_pub.publish(Twist())
            self._transition(MissionState.UNLOAD)

    # ------------------------------------------------------------------
    # Unload handling
    # ------------------------------------------------------------------
    def _begin_unload(self) -> None:
        self._unload_started = False
        self._unload_start_time = None

    def _handle_unload_state(self) -> None:
        pwm_open = self.get_parameter('unload/servo_pwm_open').get_parameter_value().double_value
        pwm_close = self.get_parameter('unload/servo_pwm_close').get_parameter_value().double_value
        hold_time = self.get_parameter('unload/servo_hold_time').get_parameter_value().double_value

        if not self._unload_started:
            self.get_logger().info(f'Opening dumpbox (PWM {pwm_open:.1f})')
            self._servo_pub.publish(Float32(data=pwm_open))
            self._unload_started = True
            self._unload_start_time = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - self._unload_start_time).nanoseconds / 1e9
        if elapsed >= hold_time:
            self.get_logger().info(f'Closing dumpbox (PWM {pwm_close:.1f})')
            self._servo_pub.publish(Float32(data=pwm_close))
            self._transition(MissionState.RETURN)

    # ------------------------------------------------------------------
    # Mission completion
    # ------------------------------------------------------------------
    def _start_return(self) -> None:
        pose = self._clone_pose(
            self._active_destination.return_pose if self._active_destination else self._default_home)
        self._send_nav_goal(pose)

    def _finalize_mission(self) -> None:
        self._request_parking_reset()
        self._requested_id = None
        self._active_destination = None
        self._marker_search_start = None
        self._parking_complete = False
        self._unload_started = False
        self._unload_start_time = None
        self._transition(MissionState.IDLE)

    def _request_parking_reset(self) -> None:
        if not self._parking_reset_client.service_is_ready():
            if not self._parking_reset_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('Parking reset service unavailable')
                return
        future = self._parking_reset_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_reset_response)

    def _on_reset_response(self, future) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'Parking reset: {response.message}')
            else:
                self.get_logger().warn(f'Parking reset rejected: {response.message}')
        except Exception as exc:
            self.get_logger().warn(f'Parking reset call failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
