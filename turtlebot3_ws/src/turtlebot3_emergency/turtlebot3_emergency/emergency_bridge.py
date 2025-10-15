#!/usr/bin/env python3
"""Emergency stop controller that toggles motor power on TurtleBot3."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class EmergencyBridge(Node):
    """Listens for emergency stop/resume commands and controls `/motor_power`."""

    def __init__(self) -> None:
        super().__init__('turtlebot3_emergency_bridge')

        self.declare_parameter('stop_topic', '/emergency_stop')
        self.declare_parameter('resume_topic', '/emergency_resume')
        self.declare_parameter('status_topic', '/emergency/state')
        self.declare_parameter('motor_service', '/motor_power')
        self.declare_parameter('service_wait_timeout', 1.0)

        self.stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        self.resume_topic = self.get_parameter('resume_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.motor_service = self.get_parameter('motor_service').get_parameter_value().string_value
        self.service_wait_timeout = (
            self.get_parameter('service_wait_timeout').get_parameter_value().double_value
        )

        qos = 10
        self.stop_sub = self.create_subscription(Bool, self.stop_topic, self._stop_callback, qos)
        self.resume_sub = self.create_subscription(Bool, self.resume_topic, self._resume_callback, qos)
        self.status_pub = self.create_publisher(Bool, self.status_topic, qos)

        self.motor_client = self.create_client(SetBool, self.motor_service)
        self._emergency_active = False
        self._pending_request = None  # 'stop' or 'resume'
        self.status_pub.publish(Bool(data=False))

        self.get_logger().info(
            f'Emergency bridge ready (stop={self.stop_topic}, resume={self.resume_topic}, '
            f'service={self.motor_service})'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _stop_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._emergency_active or self._pending_request == 'stop':
            self.get_logger().debug('Emergency stop already in effect')
            return
        if self._request_motor_power(False):
            self._pending_request = 'stop'

    def _resume_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self._emergency_active or self._pending_request == 'resume':
            self.get_logger().debug('Emergency stop already released')
            return
        if self._request_motor_power(True):
            self._pending_request = 'resume'

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _request_motor_power(self, value: bool) -> bool:
        if not self.motor_client.wait_for_service(timeout_sec=self.service_wait_timeout):
            self.get_logger().warn('Motor power service unavailable')
            return False

        request = SetBool.Request()
        request.data = value
        future = self.motor_client.call_async(request)
        future.add_done_callback(
            lambda fut, desired=value: self._handle_motor_response(fut, desired)
        )
        return True

    def _handle_motor_response(self, future, desired_state: bool) -> None:
        self._pending_request = None
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Motor power service call failed: {exc}')
            return

        if response is None or not response.success:
            message = response.message if response else 'no response'
            self.get_logger().error(f'Motor power service returned failure: {message}')
            return

        self._emergency_active = not desired_state
        self.status_pub.publish(Bool(data=self._emergency_active))

        if self._emergency_active:
            self.get_logger().info('Emergency stop engaged: motor power OFF')
        else:
            self.get_logger().info('Emergency stop released: motor power ON')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmergencyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
