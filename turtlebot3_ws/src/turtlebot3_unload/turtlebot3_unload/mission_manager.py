#!/usr/bin/env python3
"""Mission manager for coordinate-based TurtleBot3 unloading workflow."""

import enum
import math
import pathlib
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32, Int32, String

import yaml


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
    drop_pose: PoseStamped
    return_pose: PoseStamped
    staging_pose: Optional[PoseStamped] = None


class MissionState(enum.Enum):
    IDLE = enum.auto()
    NAVIGATE_STAGING = enum.auto()
    NAVIGATE_DROP = enum.auto()
    UNLOAD = enum.auto()
    RETURN = enum.auto()
    COMPLETE = enum.auto()


class MissionManager(Node):

    def __init__(self) -> None:
        super().__init__(
            'turtlebot3_unload_mission',
            automatically_declare_parameters_from_overrides=True
        )

        self._declare_if_absent('destination_topic', '/unload/destination')
        self._declare_if_absent('status_topic', '/unload/status')
        self._declare_if_absent('destination_config', 'config/destinations.yaml')
        self._declare_if_absent('dumpbox_pwm_topic', '/turtlebot3_dumpbox/servo/pwm')
        self._declare_if_absent('unload/servo_pwm_open', 1000.0)
        self._declare_if_absent('unload/servo_pwm_close', 1500.0)
        self._declare_if_absent('unload/servo_hold_time', 3.0)

        config_path = self._resolve_config_path(
            self.get_parameter('destination_config').get_parameter_value().string_value
        )
        self._destinations, self._default_home = self._load_config(config_path)

        qos = QoSProfile(depth=10)
        self._status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            qos
        )
        self._servo_pub = self.create_publisher(
            Float32,
            self.get_parameter('dumpbox_pwm_topic').value,
            qos
        )

        self.create_subscription(
            Int32,
            self.get_parameter('destination_topic').value,
            self._on_destination_request,
            qos
        )

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._state = MissionState.IDLE
        self._requested_id: Optional[int] = None
        self._active_destination: Optional[Destination] = None
        self._nav_goal_in_flight = False
        self._unload_started = False
        self._unload_start_time = None

        self._state_timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'turtlebot3_unload mission manager ready (config: {config_path})'
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
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

        home_cfg = data.get('default_home', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        default_home = build_pose(home_cfg['x'], home_cfg['y'], home_cfg['yaw'])

        for key, entry in data.get('destinations', {}).items():
            dest_id = int(key)

            drop_cfg = entry.get('drop_pose')
            if drop_cfg is None:
                raise KeyError(f'drop_pose missing for destination {dest_id}')
            drop_pose = build_pose(drop_cfg['x'], drop_cfg['y'], drop_cfg['yaw'])

            return_cfg = entry.get('return_pose')
            if return_cfg is not None:
                return_pose = build_pose(
                    return_cfg['x'],
                    return_cfg['y'],
                    return_cfg['yaw']
                )
            else:
                return_pose = build_pose(home_cfg['x'], home_cfg['y'], home_cfg['yaw'])

            staging_cfg = entry.get('staging_pose')
            staging_pose = None
            if staging_cfg:
                staging_pose = build_pose(
                    staging_cfg['x'],
                    staging_cfg['y'],
                    staging_cfg['yaw']
                )

            destinations[dest_id] = Destination(
                drop_pose=drop_pose,
                return_pose=return_pose,
                staging_pose=staging_pose
            )

        unload_cfg = data.get('unload', {})
        unload_params = []
        if 'servo_pwm_open' in unload_cfg:
            unload_params.append(Parameter(
                'unload/servo_pwm_open',
                value=float(unload_cfg['servo_pwm_open'])
            ))
        if 'servo_pwm_close' in unload_cfg:
            unload_params.append(Parameter(
                'unload/servo_pwm_close',
                value=float(unload_cfg['servo_pwm_close'])
            ))
        if 'servo_hold_time' in unload_cfg:
            unload_params.append(Parameter(
                'unload/servo_hold_time',
                value=float(unload_cfg['servo_hold_time'])
            ))
        if unload_params:
            self.set_parameters(unload_params)

        return destinations, default_home

    # ------------------------------------------------------------------
    # Mission handling
    # ------------------------------------------------------------------
    def _on_destination_request(self, msg: Int32) -> None:
        dest_id = msg.data
        if dest_id not in self._destinations:
            self.get_logger().warn(f'Unknown destination id: {dest_id}')
            return
        if self._state != MissionState.IDLE:
            self.get_logger().warn('Mission already in progress, ignoring request')
            return

        self._requested_id = dest_id
        self._active_destination = self._destinations[dest_id]

        next_state = (
            MissionState.NAVIGATE_STAGING
            if self._active_destination.staging_pose is not None
            else MissionState.NAVIGATE_DROP
        )
        self._transition(next_state, detail=f'destination={dest_id}')

    def _transition(self, new_state: MissionState, detail: Optional[str] = None) -> None:
        detail_suffix = f' ({detail})' if detail else ''
        self.get_logger().info(
            f'State: {self._state.name} -> {new_state.name}{detail_suffix}'
        )
        self._state = new_state
        self._publish_status(detail)

        if new_state == MissionState.NAVIGATE_STAGING:
            self._start_navigation(self._active_destination.staging_pose, 'staging')
        elif new_state == MissionState.NAVIGATE_DROP:
            self._start_navigation(self._active_destination.drop_pose, 'drop')
        elif new_state == MissionState.UNLOAD:
            self._begin_unload()
        elif new_state == MissionState.RETURN:
            target = self._active_destination.return_pose or self._default_home
            self._start_navigation(target, 'return')
        elif new_state == MissionState.COMPLETE:
            self._finalize_mission(detail)

    def _publish_status(self, detail: Optional[str] = None) -> None:
        dest = self._requested_id if self._requested_id is not None else -1
        detail_text = f', detail={detail}' if detail else ''
        msg = String()
        msg.data = f'state={self._state.name}, destination={dest}{detail_text}'
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def _start_navigation(self, target_pose: Optional[PoseStamped], label: str) -> None:
        if target_pose is None:
            self.get_logger().warn(f'No {label} pose provided, skipping navigation')
            self._transition(MissionState.UNLOAD if label == 'staging' else MissionState.COMPLETE,
                             detail=f'missing_{label}_pose')
            return

        pose = self._clone_pose(target_pose)
        pose.header.stamp = self.get_clock().now().to_msg()

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server unavailable')
            self._transition(MissionState.COMPLETE, 'nav_server_unavailable')
            return

        yaw_deg = math.degrees(
            2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        )
        self.get_logger().info(
            f'Sending {label} goal to ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}, {yaw_deg:.2f}°)'
        )

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_goal_in_flight = True

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_nav_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self._nav_goal_in_flight = False
            self._transition(MissionState.COMPLETE, 'goal_rejected')
            return
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._nav_goal_in_flight = False
        result = future.result()
        if result.status != 4:
            self.get_logger().warn(f'Navigation failed with status {result.status}')
            self._transition(MissionState.COMPLETE, f'nav_failed_{result.status}')
            return

        if self._state == MissionState.NAVIGATE_STAGING:
            self._transition(MissionState.NAVIGATE_DROP)
        elif self._state == MissionState.NAVIGATE_DROP:
            self._transition(MissionState.UNLOAD)
        elif self._state == MissionState.RETURN:
            self._transition(MissionState.COMPLETE, 'return_complete')

    def _on_nav_feedback(self, feedback) -> None:
        pose = feedback.feedback.current_pose.pose
        self.get_logger().debug(
            f'NAV feedback: ({pose.position.x:.2f}, {pose.position.y:.2f})'
        )

    def _clone_pose(self, template: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = template.header.frame_id or 'map'
        pose.pose = template.pose
        return pose

    # ------------------------------------------------------------------
    # Unloading
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
    # Mission timer
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._state == MissionState.UNLOAD:
            self._handle_unload_state()

    # ------------------------------------------------------------------
    # Finalisation
    # ------------------------------------------------------------------
    def _finalize_mission(self, detail: Optional[str] = None) -> None:
        self.get_logger().info('Mission completed, returning to idle')
        self._requested_id = None
        self._active_destination = None
        self._nav_goal_in_flight = False
        self._unload_started = False
        self._unload_start_time = None
        self._state = MissionState.IDLE
        self._publish_status(detail or 'completed')


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
