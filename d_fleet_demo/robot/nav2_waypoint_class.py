import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class WaypointNavigator(Node):
    """
    NavigateToPose 액션 클라이언트를 클래스 기반으로 구현한 노드.
    지정된 웨이포인트로 로봇을 이동시키는 기능을 수행합니다.
    """
    def __init__(self):
        """노드 초기화 및 액션 클라이언트 생성"""
        super().__init__('nav2_waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pinky_nav2_state = "None"

    def send_goal(self,waypoint_array):
        """
        목표 지점을 전송합니다.

        :param x: 목표 x 좌표
        :param y: 목표 y 좌표
        :param w: 목표 방향 (quaternion w)
        """
        self.get_logger().info("액션 서버를 기다리는 중...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("액션 서버를 사용할 수 없습니다. Nav2가 실행 중인지 확인하세요.")
            # rclpy.shutdown() # GUI 앱이 계속 실행되도록 종료 코드를 제거합니다.
            return

        goal_msg = NavigateToPose.Goal()
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = waypoint_array[0]
        pose.pose.position.y = waypoint_array[1]
        pose.pose.orientation.w = waypoint_array[2]
        goal_msg.pose = pose

        self.get_logger().info(f"목표 지점 전송 중: ({waypoint_array[0]}, {waypoint_array[1]})")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """액션 피드백 콜백 함수"""
        remain_dist = feedback_msg.feedback.distance_remaining
        # self.get_logger().info(f"피드백: 남은 거리 {remain_dist:.2f} m")

    def goal_response_callback(self, future):
        """목표 수락/거부 응답 콜백 함수"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            # rclpy.shutdown() # GUI 앱이 계속 실행되도록 종료 코드를 제거합니다.
            return
        
        self.get_logger().info('목표가 수락되었습니다.')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """최종 결과 콜백 함수"""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점에 성공적으로 도착했습니다!')
            self.pinky_nav2_state = "Done"
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('목표 지점 이동이 중단되었습니다.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('목표 지점 이동이 취소되었습니다.')
        else:
            self.get_logger().info(f'알 수 없는 결과 상태: {status}')

        # rclpy.shutdown() # GUI 앱이 계속 실행되도록 종료 코드를 제거합니다.


def main(args=None):
    rclpy.init(args=args)
    
    # WaypointNavigator 클래스의 인스턴스 생성
    navigator = WaypointNavigator()
    
    # 웨이포인트 (x, y, w)를 설정하여 목표 전송
    navigator.send_goal(x=0.0, y=-0.0, w=0.0)
    
    # rclpy.spin()으로 노드를 실행하고 콜백을 기다립니다.
    # get_result_callback에서 rclpy.shutdown()이 호출되면 종료됩니다.
    rclpy.spin(navigator)


if __name__ == '__main__':
    main()