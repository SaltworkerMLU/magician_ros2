import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint, ArcMotion, DrawCircle
from dobot_msgs.srv import GripperControl
import math
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor


class DrawCircleService(Node):

    def __init__(self):
        super().__init__('dobot_draw_circle')
        self._PTP_action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self._Arc_action_client = ActionClient(self, ArcMotion, 'Arc_action', callback_group=ReentrantCallbackGroup())

        self._action_server = ActionServer(
            self,
            DrawCircle,                                 # The Action Server node (this class is the Action)
            'draw_circle',                              # The name of the Action connecting the two nodes
            execute_callback=self.execute_callback,     # Result Service
            callback_group=ReentrantCallbackGroup(),    # Allows external Feedback topics such as the ones below
            goal_callback=self.goal_callback,           # Goal service
            cancel_callback=self.cancel_callback)       # A Feedback topic which can invoke Result Service to end with a response

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        self._PTP_action_client.destroy()
        self._action_server.async_cancel_all_goals()
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.position = goal_request.position
        self.radius = goal_request.radius

        self.tasks_list = [
            ["PTP", [self.position[0], self.position[1], self.position[2] + 50, 0.0], 1],
            ["PTP", [self.position[0], self.position[1], self.position[2], 0.0], 2],
            ["Arc", [self.position[0] + self.radius,       self.position[1] + self.radius, self.position[2], 0.0], 
                    [self.position[0] + 2.0 * self.radius, self.position[1],               self.position[2], 0.0]],
            ["Arc", [self.position[0] + self.radius,       self.position[1] - self.radius, self.position[2], 0.0], 
                    [self.position[0],                     self.position[1],               self.position[2], 0.0]],
            #["Arc", [187.5, 28.35, -74.0, 0.0], [212.5, 28.35, -74.0, 0.0]],
            #["Arc", [225.0, 50.0, -74.0, 0.0], [212.5, 71.65, -74.0, 0.0]],
            #["Arc", [187.5, 71.65, -74.0, 0.0], [X_position, 50.0, -74.0, 0.0]],
            ["PTP", [self.position[0], self.position[1], self.position[2] + 50, 0.0], 2],
        ]

        self.goal_num = 0

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
            """Execute a goal."""
            result = DrawCircle.Result()

            self.complete = False
            self.mission()

            while self.complete != True:
                pass

            goal_handle.succeed()

            result.success = True

            return result

    def mission(self):
        if self.goal_num > len(self.tasks_list)-1:
            self.complete = True
            return 
            rclpy.shutdown()
            sys.exit()
        else:
            self.get_logger().info('*** TASK NUM ***: {0}'.format(self.goal_num))

        if self.tasks_list[self.goal_num][0] == "Arc":
            self.send_Arc_goal(*self.tasks_list[self.goal_num][1:])
            self.goal_num = self.goal_num + 1
        elif self.tasks_list[self.goal_num][0] == "PTP":
            self.send_PTP_goal(*self.tasks_list[self.goal_num][1:])
            self.goal_num = self.goal_num + 1

    def send_PTP_goal(self, _target, _type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type

        self._PTP_action_client.wait_for_server()

        self._send_goal_future = self._PTP_action_client.send_goal_async(goal_msg) # , feedback_callback=self.feedback_callback

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_Arc_goal(self, _circumference_point, _ending_point):
        goal_msg = ArcMotion.Goal()
        goal_msg.circumference_point = _circumference_point
        goal_msg.ending_point = _ending_point

        self._Arc_action_client.wait_for_server()

        self._send_goal_future = self._Arc_action_client.send_goal_async(goal_msg) # , feedback_callback=self.feedback_callback

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result of action call: {0}'.format(result))
            #self.execute_callback()
            self.mission()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    service_server = DrawCircleService()

    executor = MultiThreadedExecutor()

    rclpy.spin(service_server, executor=executor)
    
    service_server.destroy()

if __name__ == '__main__':
    main()
