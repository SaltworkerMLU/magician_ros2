import time
import math
from dobot_msgs.action import ArcMotion
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 
import tf_transformations
from dobot_driver.dobot_handle import bot
from ._PTP_params_class import declare_PTP_params
from rcl_interfaces.msg import SetParametersResult
from threading import Thread
from dobot_msgs.srv import EvaluateARCTrajectory
from dobot_msgs.msg import DobotAlarmCodes
from std_msgs.msg import Float64MultiArray
from dobot_nodes._callback_action import goal_callback_action, cancel_callback_action, execute_callback_action

class DobotArcServer(Node):

    # Make functions into class methods
    execute_callback_action = execute_callback_action
    goal_callback_action = goal_callback_action
    cancel_callback = cancel_callback_action

    def __init__(self):
        super().__init__('dobot_ARC_server')  # Call any existing __init__ methods using super() as 'dobot_Arc_server'

        self._action_server = ActionServer(             # The Action Server node (this class is the Action)
            self,                                       # The name of the Action connecting the two nodes
            ArcMotion,                                  # Result Service
            'Arc_action',                               # Allows external Feedback topics such as the ones below
            execute_callback=self.execute_callback,     # Goal service
            callback_group=ReentrantCallbackGroup(),    # A Feedback topic which can invoke Result Service to end with a response
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.subscription_TCP = self.create_subscription(
            Float64MultiArray,
            'dobot_pose_raw',
            self.arc_position_callback,
            10)
        
        self.subscription_alarms = self.create_subscription(
            DobotAlarmCodes,
            'dobot_alarms',
            self.active_alarms_callback,
            10)
        
        self.client_validate_goal = self.create_client(srv_type = EvaluateARCTrajectory, 
                                                        srv_name = 'dobot_ARC_validation_service', 
                                                        callback_group=ReentrantCallbackGroup())
        while not self.client_validate_goal.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trajectory validation service not available, waiting again...')

        self.motion_type = None 
        self.pc = []
        self.pf = [] 
        self.pose_arr = []
        self.dobot_pose = [] 
        self.mode_ACK = False

        self.motion_types_list = [1]
        self.motion_type = 1

        self.joints_sorted = False
        self.cartesian_sorted = False

        self.ready_to_set_second_part = False

        self.active_alarms = False

        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def arc_position_callback(self, msg):
        self.dobot_pose = [float(msg.data[0])*1000, float(msg.data[1])*1000, float(msg.data[2])*1000, float(msg.data[3])]
        self.mode_ACK = True

    def active_alarms_callback(self, msg):
        if not msg.alarms_list:
            self.active_alarms = False
        else:
            self.active_alarms = True

    def parameters_callback(self, params):
        for param in params:
            if param.name in self.joint_params_names:
                self.joint_params_dict[param.name] = param.value
                if self.allow_dynamic_reconfigure_joints:
                    self.send_joint_parameters()
               
            elif param.name in self.cartesian_params_names:
                self.cartesian_params_dict[param.name] = param.value
                if self.allow_dynamic_reconfigure_cartesian:
                    self.send_cartesian_parameters()

            else:
                return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
  
    def send_request_check_trajectory(self, p0, pc, pf):
        self.req_validate = EvaluateARCTrajectory.Request()
        self.req_validate.p0 = p0
        self.req_validate.pc = pc
        self.req_validate.pf = pf
        self.future = self.client_validate_goal.call_async(self.req_validate)
        while not self.future.done():
            pass
        return self.future.result()
    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.p0 = self.dobot_pose
        self.pc = goal_request.pc
        self.pf = goal_request.pf
        
        # Check if trajectory is feasible within robot workspace
        validation_response = self.send_request_check_trajectory(self.p0,
                                                                 self.pc,
                                                                 self.pf)
        if validation_response.is_valid == False:
            self.get_logger().warn("Trajectory rejected: {0}".format(validation_response))
            return GoalResponse.REJECT

        self.get_logger().info("Result of calling validation service: is valid? {0}, description: {1}".format(validation_response.is_valid, validation_response.message))

        self.get_logger().info('Checkpoint: {0}'.format(self.pc))
        self.get_logger().info('Goal: {0}'.format(self.pf))

        _goalResponse = self.goal_callback_action(goal_request)

        return _goalResponse
    
    async def execute_callback(self, goal_handle):
        bot.set_arc_command([self.pc[0], 
                             self.pc[1],
                             self.pc[2],
                             self.pc[3]], 
                            [self.pf[0], 
                             self.pf[1], 
                             self.pf[2], 
                             self.pf[3]])
        
        feedback_msg = ArcMotion.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]
        self.pose_arr = []

        result = ArcMotion.Result()

        self.target = self.pf

        self.execute_callback_action(goal_handle, result, feedback_msg)

        result.achieved_pose = self.dobot_pose
        self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

        return result

def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = DobotArcServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
