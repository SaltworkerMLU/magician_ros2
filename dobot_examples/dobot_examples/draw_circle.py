import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint, ArcMotion
from dobot_msgs.srv import GripperControl
import math
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor


class DrawCircle(Node):

    """
    ros2 run dobot_demos test_draw_circle --ros-args --params-file $(ros2 pkg prefix dobot_demos)/share/dobot_demos/config/draw_circle.yaml
    """

    def __init__(self):
        super().__init__('dobot_draw_circle')
        self._PTP_action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self._Arc_action_client = ActionClient(self, ArcMotion, 'Arc_action', callback_group=ReentrantCallbackGroup())

        # Default parameters
        self.position = [175.0, 50.0, -74.0]
        self.radius = 25.0
        
        self.add_on_set_parameters_callback(self.parameters_callback) # FIRST register callback

        self.declare_parameter('position',rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('radius',rclpy.Parameter.Type.DOUBLE)

        # Parameters defined by config/draw_circle.yaml
        self.position = self.get_parameter('position').value
        self.radius = self.get_parameter('radius').value

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

    def parameters_callback(self, params):
        self.get_logger().info("Hello yes parameters go brr")
        for param in params:
            if param.name == 'position':
                self.position[0] = param.value[0]
                self.position[1] = param.value[1]
                self.position[2] = param.value[2]
                return SetParametersResult(successful=True)
            elif param.name == 'radius':
                self.radius = param.value
                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(successful=False)

    def execute(self):
            if self.goal_num > len(self.tasks_list)-1:
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
            self.execute()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = DrawCircle()

    action_client.execute()

    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()


