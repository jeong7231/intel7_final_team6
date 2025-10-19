import os
import time
import math
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from std_msgs.msg import Int32, String, Float32, Empty
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class State:
    IDLE = 0
    NAVIGATE = 1
    UNLOAD_OPEN = 2
    UNLOAD_WAIT = 3
    UNLOAD_CLOSE = 4
    RETURN = 5


class UnloadNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_unload')

        # 1) YAML 경로 파라미터만 선언
        self.declare_parameter('config_file', '')

        # 2) 경로 결정
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            cfg_path = os.path.join(
                get_package_share_directory('turtlebot3_unload'),
                'config', 'destinations.yaml'
            )
        if not os.path.isfile(cfg_path):
            self.get_logger().error(f'config YAML not found: {cfg_path}')
            raise SystemExit(1)

        # 3) YAML 로드
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}

        # 4) 내부 설정 적용
        self.dest_cfg = cfg.get('destinations', {})
        self.home = cfg.get('home', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

        un_cfg = cfg.get('unload', {})
        self.dump_open = float(un_cfg.get('pwm_open', 1000.0))
        self.dump_close = float(un_cfg.get('pwm_close', 1500.0))
        self.dump_hold = float(un_cfg.get('hold_sec', 3.0))

        topics = cfg.get('topics', {})
        topic_dest = topics.get('dest', '/unload/destination')
        topic_status = topics.get('status', '/unload/status')
        topic_dump = topics.get('dump_pwm', '/turtlebot3_dumpbox/servo/pwm')
        topic_return = topics.get('returned', '/unload/returned')

        # 퍼블리셔
        self.pub_status = self.create_publisher(String, topic_status, 10)
        q_event = QoSProfile(depth=1)
        q_event.reliability = ReliabilityPolicy.RELIABLE
        q_event.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_returned = self.create_publisher(Empty, topic_return, q_event)
        self.pub_dump = self.create_publisher(Float32, topic_dump, 10)

        # 구독
        self.create_subscription(Int32, topic_dest, self.on_dest, 10)

        # 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 상태
        self.state = State.IDLE
        self.current_dest = None
        self.report('IDLE')

    # 유틸
    def report(self, s: str):
        self.pub_status.publish(String(data=s))
        self.get_logger().info(s)

    def build_pose(self, x, y, yaw) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quat(float(yaw))
        return ps

    def nav_to(self, x, y, yaw, tag: str) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.report('ERROR:NAV_SERVER_UNAVAILABLE')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.build_pose(x, y, yaw)
        self.report(f'{tag}:START')

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.report(
                f"{tag}:FEEDBACK dist={getattr(feedback.feedback, 'distance_remaining', float('nan')):.3f}"
            ))

        try:
            gh = self._wait_for_future(send_future)
        except Exception as exc:  # pylint: disable=broad-except
            self.report(f'ERROR:NAV_GOAL_EXCEPTION {exc}')
            return False

        if gh is None or not gh.accepted:
            self.report('ERROR:NAV_GOAL_REJECTED')
            return False

        self.report(f'{tag}:GOAL_ACCEPTED')

        try:
            res = self._wait_for_future(gh.get_result_async())
        except Exception as exc:  # pylint: disable=broad-except
            self.report(f'ERROR:NAV_RESULT_EXCEPTION {exc}')
            return False
        if res is None:
            self.report('ERROR:NAV_NO_RESULT')
            return False

        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.report(f'{tag}:RESULT status=SUCCEEDED')
            self.report(f'{tag}:ARRIVED')
            return True

        if res.status == GoalStatus.STATUS_ABORTED:
            self.report('ERROR:NAV_ABORTED')
            return False

        if res.status == GoalStatus.STATUS_CANCELED:
            self.report('ERROR:NAV_CANCELED')
            return False

        self.report(f'ERROR:NAV_UNKNOWN_STATUS status={res.status}')
        return False

    def _wait_for_future(self, future):
        event = threading.Event()

        def _on_done(_):
            event.set()

        future.add_done_callback(_on_done)

        while rclpy.ok():
            if future.done():
                break
            # wait briefly to avoid busy-waiting but still allow interrupt
            event.wait(0.05)

        if not future.done():
            raise RuntimeError('Future did not complete before shutdown')

        if future.cancelled():
            raise RuntimeError('Future was cancelled')

        if future.exception() is not None:
            raise future.exception()

        return future.result()

    def sleep_sec(self, sec: float):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            time.sleep(0.05)

    # 콜백
    def on_dest(self, msg: Int32):
        code = str(msg.data)
        if code not in self.dest_cfg:
            self.report(f'ERROR:INVALID_DEST dest={code}')
            return
        if self.state != State.IDLE:
            self.report('WARN:BUSY')
            return
        self.current_dest = code
        threading.Thread(target=self.sequence, daemon=True).start()

    # 시퀀스
    def sequence(self):
        # 1) 목적지 이동
        self.state = State.NAVIGATE
        d = self.dest_cfg[self.current_dest]['drop_pose']
        if not self.nav_to(d['x'], d['y'], d['yaw'], 'NAVIGATE'):
            self.state = State.IDLE
            return

        # 2) 언로드: Open -> Wait -> Close
        self.state = State.UNLOAD_OPEN
        self.pub_dump.publish(Float32(data=self.dump_open))
        self.report(f'UNLOAD:OPEN pwm={self.dump_open:.1f}')

        self.state = State.UNLOAD_WAIT
        self.sleep_sec(self.dump_hold)
        self.report(f'UNLOAD:WAIT sec={self.dump_hold:.1f}')

        self.state = State.UNLOAD_CLOSE
        self.pub_dump.publish(Float32(data=self.dump_close))
        self.report(f'UNLOAD:CLOSE pwm={self.dump_close:.1f}')

        # 3) 홈 복귀
        self.state = State.RETURN
        h = self.home
        if not self.nav_to(h['x'], h['y'], h['yaw'], 'RETURN'):
            self.state = State.IDLE
            return

        # 4) 완료 신호
        self.pub_returned.publish(Empty())
        self.report('COMPLETE')
        self.state = State.IDLE
        self.report('IDLE')


def main():
    rclpy.init()
    node = UnloadNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
