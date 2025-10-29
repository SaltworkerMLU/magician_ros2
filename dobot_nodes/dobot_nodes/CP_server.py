import time
import math
from dobot_msgs.action import DrawCircle
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
from dobot_msgs.srv import EvaluateArcTrajectory, EvaluatePTPTrajectory
from dobot_msgs.msg import DobotAlarmCodes
from std_msgs.msg import Float64MultiArray
from dobot_nodes._callback_action import goal_callback_action, cancel_callback_action, execute_callback_action

class DobotCPServer(Node):

    # Make functions into class methods
    execute_callback_action = execute_callback_action
    goal_callback_action = goal_callback_action
    cancel_callback = cancel_callback_action

    def __init__(self):
        super().__init__('dobot_CP_server')  # Call any existing __init__ methods using super() as 'dobot_Arc_server'

        self._action_server = ActionServer(             # The Action Server node (this class is the Action)
            self,                                       # The name of the Action connecting the two nodes
            DrawCircle,                                  # Result Service
            'CP_action',                               # Allows external Feedback topics such as the ones below
            execute_callback=self.execute_callback,     # Goal service
            callback_group=ReentrantCallbackGroup(),    # A Feedback topic which can invoke Result Service to end with a response
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.subscription_joints = self.create_subscription(
            JointState,
            'dobot_joint_states',
            self.joints_positions_callback,
            10)

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
        
        self.client_validate_goal = self.create_client(srv_type = EvaluatePTPTrajectory, 
                                                        srv_name = 'dobot_PTP_validation_service', 
                                                        callback_group=ReentrantCallbackGroup())
        while not self.client_validate_goal.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trajectory validation service not available, waiting again...')

        self.motion_type = 1
        self.circumference_point = []
        self.ending_point = [] 
        self.pose_arr = []
        self.dobot_pose = [] 
        self.mode_ACK = False

        self.joints_sorted = False
        self.cartesian_sorted = False

        self.ready_to_set_second_part = False

        self.active_alarms = False

        self.add_on_set_parameters_callback(self.parameters_callback)

    def joints_positions_callback(self, msg):
        if self.motion_type in [3, 4, 5, 6]:
            self.dobot_pose = [math.degrees(msg.position[0]), math.degrees(msg.position[1]), math.degrees(msg.position[2]), math.degrees(msg.position[3])]
            self.mode_ACK = True


    def tcp_position_callback(self, msg):
        if self.motion_type in [0, 1, 2, 7, 8, 9]:
            self.dobot_pose = [float(msg.data[0])*1000, float(msg.data[1])*1000, float(msg.data[2])*1000, float(msg.data[3])]
            self.mode_ACK = True
    
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
    
    def that_function(self):
        [start_x, start_y, start_z, start_r] = self.target

        bot.set_continous_trajectory_real_time_params(20, 100, 10)

        # Draw about half an arch as a single path
        bot.stop_queue()
        #bot.clear_queue()
        #bot.set_continous_trajectory_command(1, start_x + 25, start_y + 25, start_z, 0.5) # start_r
        steps = 24
        scale = 25
        for i in range(steps + 2):
            x = math.cos((2*math.pi / steps) * i)
            y = math.sin((2*math.pi / steps) * i)

            if i == 0:
                bot.set_point_to_point_command(self.motion_type, start_x + x * scale, start_y + y * scale, self.z_level+25, self.target[3], queue=True)

            # Absolute movement
            bot.set_continous_trajectory_command(1, start_x + x * scale, start_y + y * scale, self.z_level, 0.5, queue=True) # start_r
        
        #bot.set_continous_trajectory_command(1, start_x, start_y, start_z, 0.5, queue=True)

        bot.set_point_to_point_command(self.motion_type, self.target[0], self.target[1], self.target[2], self.target[3], queue=True)

        bot.start_queue()

    async def execute_callback(self, goal_handle):
        #Execute a goal.
        self.get_logger().info('Executing goal...')

        self.that_function()
        
        feedback_msg = DrawCircle.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]
        self.pose_arr = []

        result = DrawCircle.Result()

        #result.achieved_pose = self.target

        self.execute_callback_action(goal_handle, result, feedback_msg)

        return result
  
    def send_request_check_trajectory(self, target):
        self.req_validate = EvaluatePTPTrajectory.Request()
        self.req_validate.target = target
        self.req_validate.motion_type = 1
        self.future = self.client_validate_goal.call_async(self.req_validate)
        while not self.future.done():
            pass
        return self.future.result()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.radius = goal_request.radius
        self.target_pose = goal_request.target_pose
        self.z_level = goal_request.z_level

        self.target = self.target_pose

        # Check if trajectory is feasible within robot workspace
        validation_response = self.send_request_check_trajectory(self.target_pose)
        if validation_response.is_valid == False:
            self.get_logger().warn("Goal rejected: {0}".format(validation_response))
            return GoalResponse.REJECT
        
        self.get_logger().info("Result of calling validation service: is valid? {0}, description: {1}".format(validation_response.is_valid, validation_response.message))
        
        self.get_logger().info('Goal: {0}'.format(self.target))
        self.get_logger().info('Mode: {0}'.format(self.motion_type))
        
        _goalResponse = self.goal_callback_action(goal_request)

        return _goalResponse
    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = DobotCPServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
