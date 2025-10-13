#!/usr/bin/env python3
"""Mission coordination node for the TurtleBot3 auto unload workflow."""

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
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

import tf2_ros

from ament_index_python.packages import get_package_share_directory


def build_pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


@dataclass
class DestinationConfig:
    marker_id: int
    staging_pose: PoseStamped
    drop_pose: PoseStamped
    return_pose: PoseStamped


class MissionState(enum.Enum):
    IDLE = enum.auto()
    NAVIGATE_TO_STAGING = enum.auto()
    SEARCH_MARKER = enum.auto()
    AUTO_PARK = enum.auto()
    DUMP = enum.auto()
    RETURN_HOME = enum.auto()
    COMPLETE = enum.auto()


class MissionManager(Node):
    """State machine that sequences navigation, marker search, parking, and unloading."""

    def __init__(self) -> None:
        super().__init__('auto_unload_mission')

        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('destination_topic', '/auto_unload/destination')
        self.declare_parameter('status_topic', '/auto_unload/status')
        self.declare_parameter('destination_config', 'config/destinations.yaml')
        self.declare_parameter('parking_status_topic', '/automatic_parking/status')
        self.declare_parameter('parking_reset_service', '/automatic_parking/reset')
        self.declare_parameter('target_marker_topic', '/target_marker_id')
        self.declare_parameter('marker_search/rotation_speed', 0.3)
        self.declare_parameter('marker_search/max_search_time', 30.0)
        self.declare_parameter('dumpbox_service', 'turtlebot3_dumpbox/servo/apply_default')

        config_path = self._resolve_config_path(
            self.get_parameter('destination_config').get_parameter_value().string_value)
        self._destinations, self._default_home = self._load_destination_config(config_path)

        self._state = MissionState.IDLE
        self._requested_destination: Optional[int] = None
        self._active_destination: Optional[DestinationConfig] = None

        qos = QoSProfile(depth=10)
        destination_topic = self.get_parameter('destination_topic').value
        self.create_subscription(Int32, destination_topic, self._on_destination_request, qos)

        status_topic = self.get_parameter('status_topic').value
        self._status_pub = self.create_publisher(String, status_topic, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._target_marker_pub = self.create_publisher(Int32, self.get_parameter('target_marker_topic').value, qos)

        self._parking_status_topic = self.get_parameter('parking_status_topic').value
        self.create_subscription(String, self._parking_status_topic, self._on_parking_status, qos)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._dumpbox_client = self.create_client(Trigger, self.get_parameter('dumpbox_service').value)
        self._parking_reset_client = self.create_client(
            Trigger, self.get_parameter('parking_reset_service').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_detected = False
        self._nav_goal_in_flight = False
        self._marker_search_start = None
        self._parking_complete = False

        self._state_timer = self.create_timer(0.5, self._run_state_machine)
        self.get_logger().info('Auto unload mission manager ready (config: %s)', config_path)

    def _resolve_config_path(self, config_param: str) -> pathlib.Path:
        candidate = pathlib.Path(config_param)
        if candidate.is_file():
            return candidate
        package_share = pathlib.Path(get_package_share_directory('turtlebot3_auto_unload'))
        resolved = package_share / config_param
        if not resolved.is_file():
            raise FileNotFoundError(f'Destination config not found: {resolved}')
        return resolved

    def _load_destination_config(self, path: pathlib.Path) -> (Dict[int, DestinationConfig], PoseStamped):
        import yaml

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        destinations: Dict[int, DestinationConfig] = {}
        for key, node in data.get('destinations', {}).items():
            dest_id = int(key)
            marker_id = int(node['marker_id'])
            staging = build_pose_stamped(node['staging_pose']['x'], node['staging_pose']['y'], node['staging_pose']['yaw'])
            drop = build_pose_stamped(node['drop_pose']['x'], node['drop_pose']['y'], node['drop_pose']['yaw'])
            ret = build_pose_stamped(node['return_pose']['x'], node['return_pose']['y'], node['return_pose']['yaw'])
            destinations[dest_id] = DestinationConfig(marker_id=marker_id, staging_pose=staging, drop_pose=drop, return_pose=ret)

        home_cfg = data.get('default_home', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        default_home = build_pose_stamped(home_cfg['x'], home_cfg['y'], home_cfg['yaw'])

        search_cfg = data.get('marker_search', {})
        override_params = []
        if 'rotation_speed' in search_cfg:
            override_params.append(Parameter(
                'marker_search/rotation_speed',
                Parameter.Type.DOUBLE,
                float(search_cfg['rotation_speed'])
            ))
        if 'max_search_time' in search_cfg:
            override_params.append(Parameter(
                'marker_search/max_search_time',
                Parameter.Type.DOUBLE,
                float(search_cfg['max_search_time'])
            ))
        if override_params:
            self.set_parameters(override_params)

        return destinations, default_home

    def _on_destination_request(self, msg: Int32) -> None:
        dest_id = msg.data
        if dest_id not in self._destinations:
            self.get_logger().warn('Received unknown destination id %d', dest_id)
            return
        self._requested_destination = dest_id
        if self._state == MissionState.IDLE:
            self._transition_to(MissionState.NAVIGATE_TO_STAGING)

    def _on_parking_status(self, msg: String) -> None:
        text = msg.data.lower()
        if 'complete' in text or 'finished' in text:
            self._parking_complete = True
        if 'searching' in text:
            self._marker_detected = False

    def _transition_to(self, new_state: MissionState) -> None:
        self.get_logger().info('State transition: %s -> %s', self._state.name, new_state.name)
        self._state = new_state
        self._publish_status()
        if new_state == MissionState.NAVIGATE_TO_STAGING:
            self._request_parking_reset()
            self._start_navigation(goal_type='staging')
        elif new_state == MissionState.SEARCH_MARKER:
            self._begin_marker_search()
        elif new_state == MissionState.AUTO_PARK:
            self._start_auto_parking()
        elif new_state == MissionState.DUMP:
            self._trigger_dumpbox()
        elif new_state == MissionState.RETURN_HOME:
            self._start_navigation(goal_type='return')

    def _publish_status(self) -> None:
        msg = String()
        dest = self._active_destination.marker_id if self._active_destination else -1
        msg.data = f'state={self._state.name}, marker={dest}'
        self._status_pub.publish(msg)

    def _clone_pose(self, template: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = template.header.frame_id or 'map'
        pose.pose.position.x = template.pose.position.x
        pose.pose.position.y = template.pose.position.y
        pose.pose.position.z = template.pose.position.z
        pose.pose.orientation.x = template.pose.orientation.x
        pose.pose.orientation.y = template.pose.orientation.y
        pose.pose.orientation.z = template.pose.orientation.z
        pose.pose.orientation.w = template.pose.orientation.w
        return pose

    def _start_navigation(self, goal_type: str) -> None:
        if self._requested_destination is None:
            self.get_logger().warn('Navigation requested without destination')
            return
        self._active_destination = self._destinations[self._requested_destination]
        if goal_type == 'staging':
            template = self._active_destination.staging_pose
        elif goal_type == 'return':
            template = self._active_destination.return_pose
        else:
            template = self._active_destination.drop_pose
        pose = self._clone_pose(template)
        pose.header.stamp = self.get_clock().now().to_msg()
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server unavailable')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info('Sending navigation goal (%s)', goal_type)
        self._nav_goal_in_flight = True
        self._nav_client.send_goal_async(goal_msg, feedback_callback=self._on_nav_feedback).add_done_callback(
            self._on_nav_goal_response)

    def _request_parking_reset(self) -> None:
        if not self._parking_reset_client.service_is_ready():
            if not self._parking_reset_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('Parking reset service unavailable')
                return
        request = Trigger.Request()
        future = self._parking_reset_client.call_async(request)
        future.add_done_callback(self._on_parking_reset_response)

    def _on_parking_reset_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn('Parking reset call failed: %s', exc)
            return
        if response.success:
            self.get_logger().debug('Parking reset: %s', response.message)
        else:
            self.get_logger().warn('Parking reset rejected: %s', response.message)

    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self._nav_goal_in_flight = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self._nav_goal_in_flight = False
        if result.status == 4:
            self.get_logger().info('Navigation goal succeeded')
            if self._state == MissionState.NAVIGATE_TO_STAGING:
                self._transition_to(MissionState.SEARCH_MARKER)
            elif self._state == MissionState.RETURN_HOME:
                self._transition_to(MissionState.COMPLETE)
        else:
            self.get_logger().warn('Navigation goal failed with status %s', result.status)

    def _on_nav_feedback(self, feedback):
        pose = feedback.feedback.current_pose.pose
        self.get_logger().debug('NAV feedback pose: (%.2f, %.2f)', pose.position.x, pose.position.y)

    def _begin_marker_search(self) -> None:
        self._marker_detected = False
        self._marker_search_start = self.get_clock().now()
        self.get_logger().info('Starting marker search (marker id %d)', self._active_destination.marker_id)
        self._target_marker_pub.publish(Int32(data=self._active_destination.marker_id))

    def _start_auto_parking(self) -> None:
        self._parking_complete = False
        self.get_logger().info('Triggering automatic parking')
        self._target_marker_pub.publish(Int32(data=self._active_destination.marker_id))

    def _trigger_dumpbox(self) -> None:
        if not self._dumpbox_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Dumpbox service unavailable')
            return
        request = Trigger.Request()
        future = self._dumpbox_client.call_async(request)
        future.add_done_callback(lambda _: self.get_logger().info('Dumpbox service call completed'))

    def _run_state_machine(self) -> None:
        if self._state == MissionState.IDLE:
            return
        if self._state == MissionState.SEARCH_MARKER:
            self._handle_marker_search()
        elif self._state == MissionState.AUTO_PARK:
            self._handle_auto_park()
        elif self._state == MissionState.DUMP:
            self._transition_to(MissionState.RETURN_HOME)
        elif self._state == MissionState.COMPLETE:
            self._reset_mission()

    def _handle_marker_search(self) -> None:
        if self._check_marker_visible():
            self.get_logger().info('Marker detected, handing off to auto parking')
            self._marker_detected = True
            self._cmd_vel_pub.publish(Twist())
            self._transition_to(MissionState.AUTO_PARK)
            return
        if self._marker_search_start is None:
            self._marker_search_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._marker_search_start).nanoseconds / 1e9
        timeout = self.get_parameter('marker_search/max_search_time').value
        if elapsed > timeout:
            self.get_logger().warn('Marker search timeout')
            self._cmd_vel_pub.publish(Twist())
            self._request_parking_reset()
            self._transition_to(MissionState.RETURN_HOME)
            return
        twist = Twist()
        twist.angular.z = self.get_parameter('marker_search/rotation_speed').value
        self._cmd_vel_pub.publish(twist)

    def _handle_auto_park(self) -> None:
        if self._parking_complete:
            self.get_logger().info('Parking reported complete, proceeding to dump')
            stop_cmd = Twist()
            self._cmd_vel_pub.publish(stop_cmd)
            self._transition_to(MissionState.DUMP)

    def _check_marker_visible(self) -> bool:
        try:
            marker_frame = f'ar_marker_{self._active_destination.marker_id}'
            self._tf_buffer.lookup_transform('base_link', marker_frame, rclpy.time.Time())
            return True
        except Exception:
            return False

    def _reset_mission(self) -> None:
        self.get_logger().info('Mission complete, returning to idle')
        self._request_parking_reset()
        self._requested_destination = None
        self._active_destination = None
        self._parking_complete = False
        self._marker_detected = False
        self._marker_search_start = None
        self._transition_to(MissionState.IDLE)


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
