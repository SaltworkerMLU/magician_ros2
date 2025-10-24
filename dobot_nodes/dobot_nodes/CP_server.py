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
from dobot_msgs.srv import EvaluateArcTrajectory, EvaluatePTPTrajectory
from dobot_msgs.msg import DobotAlarmCodes
from std_msgs.msg import Float64MultiArray

class DobotArcServer(Node):

    def __init__(self):
        super().__init__('dobot_CP_server')  # Call any existing __init__ methods using super() as 'dobot_Arc_server'

        self._action_server = ActionServer(             # The Action Server node (this class is the Action)
            self,                                       # The name of the Action connecting the two nodes
            ArcMotion,                                  # Result Service
            'CP_action',                               # Allows external Feedback topics such as the ones below
            execute_callback=self.execute_callback,     # Goal service
            callback_group=ReentrantCallbackGroup(),    # A Feedback topic which can invoke Result Service to end with a response
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        """self.subscription_joints = self.create_subscription(
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
            10)"""
        
        self.client_validate_goal = self.create_client(srv_type = EvaluatePTPTrajectory, 
                                                        srv_name = 'dobot_PTP_validation_service', 
                                                        callback_group=ReentrantCallbackGroup())
        while not self.client_validate_goal.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trajectory validation service not available, waiting again...')

        self.motion_type = None 
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

    @staticmethod
    def is_goal_reached(target_pose, current_pose, threshold):
        for i in range(4):
            if abs(target_pose[i]-current_pose[i]) > threshold:
                return False
        return True
    
    @staticmethod
    def is_pose_stable(pose_arr):
        if len(pose_arr) >= 2:
            if pose_arr[-1] == pose_arr[-2]:
                return True
        return False

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        [start_x, start_y, start_z, start_r] = self.circumference_point

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

            # Absolute movement
            bot.set_continous_trajectory_command(1, start_x + x * scale, start_y + y * scale, start_z, 0.5) # start_r

        bot.start_queue()

        """bot.set_arc_command([self.circumference_point[0], 
                             self.circumference_point[1],
                             self.circumference_point[2],
                             self.circumference_point[3]], 
                            [self.ending_point[0], 
                             self.ending_point[1], 
                             self.ending_point[2], 
                             self.ending_point[3]])"""
        
        feedback_msg = ArcMotion.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]
        self.pose_arr = []

        result = ArcMotion.Result()

        """# Start executing the action
        while not (DobotArcServer.is_goal_reached(self.ending_point, self.dobot_pose, 0.2) 
                   and DobotArcServer.is_pose_stable(self.pose_arr)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                bot.stop_queue(force=True) 
                bot.clear_queue()
                bot.start_queue()
                self.get_logger().info('Goal canceled')
                result.achieved_pose  = self.dobot_pose
                return result


            # Update sequence
            feedback_msg.current_pose = self.dobot_pose
            self.pose_arr.append(self.dobot_pose)

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.current_pose))
            self.get_logger().info('Queue: {0}'.format(str(bot.get_current_queue_index())))
            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(0.1)"""


        goal_handle.succeed()


        result.achieved_pose  = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

        return result
  
    def send_request_check_trajectory(self, target):
        self.req_validate = EvaluatePTPTrajectory.Request()
        self.req_validate.target = target
        self.req_validate.motion_type = 1
        self.future = self.client_validate_goal.call_async(self.req_validate)
        while not self.future.done():
            pass
        return self.future.result()

    @staticmethod
    def is_ratio_valid(ratio):
        if 0.0 < ratio <= 1.0 and int(ratio * 100) != 0:
            return True
        return False
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.circumference_point = goal_request.circumference_point
        self.ending_point = goal_request.ending_point
        
        # Check if there are active alarms (if the LED diode lights up red) 
        if self.active_alarms:
            self.get_logger().warn("Goal rejected because of active alarms (LED diode in the robot base lights up red)")
            return GoalResponse.REJECT

        # Check if trajectory is feasible within robot workspace
        validation_response1 = self.send_request_check_trajectory(self.circumference_point)
        if validation_response1.is_valid == False:
            self.get_logger().warn("Goal rejected: {0}".format(validation_response1))
            return GoalResponse.REJECT
        
        validation_response2 = self.send_request_check_trajectory(self.ending_point)
        if validation_response2.is_valid == False:
            self.get_logger().warn("Goal rejected: {0}".format(validation_response2))
            return GoalResponse.REJECT

        self.get_logger().info("Result of calling validation service: is valid? {0}, description: {1} + {2}".format(
            validation_response1.is_valid and validation_response2.is_valid, validation_response1.message, validation_response2.message))


        # Check if velocity and acceleration ratios are valid
        if DobotArcServer.is_ratio_valid(goal_request.velocity_ratio) and DobotArcServer.is_ratio_valid(goal_request.acceleration_ratio):
            vel_ratio = int(goal_request.velocity_ratio * 100)
            acc_ratio = int(goal_request.acceleration_ratio * 100)
            self.get_logger().info("Vel ratio: {0}".format(vel_ratio))
            self.get_logger().info("Acc ratio: {0}".format(acc_ratio))
            bot.set_arc_params(vel_ratio, vel_ratio, acc_ratio, acc_ratio)
        else:
            self.get_logger().info('Wrong ratio in action goal field')
            return GoalResponse.REJECT

        # Wait until robot mode, e.g. PTP MOVEL TCP-coords, is set
        #while not self.mode_ACK:
        #    pass

        self.get_logger().info('Goal: {0}'.format(self.ending_point))
        self.get_logger().info('Mode: {0}'.format(self.circumference_point))
        self.get_logger().info('Received goal request')

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

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
