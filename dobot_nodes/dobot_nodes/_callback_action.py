from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
from dobot_msgs.action import PointToPoint, ArcMotion
import time

def is_ratio_valid(ratio):
    if 0.0 < ratio <= 1.0 and int(ratio * 100) != 0:
        return True
    return False

def goal_callback_action(self, goal_request):
    """Accept or reject a client request to begin an action."""

    # Check if there are active alarms (if the LED diode lights up red) 
    if self.active_alarms:
        self.get_logger().warn("Goal rejected because of active alarms (LED diode in the robot base lights up red)")
        return GoalResponse.REJECT

    # Check if velocity and acceleration ratios are valid
    if ( is_ratio_valid(goal_request.velocity_ratio) and 
         is_ratio_valid(goal_request.acceleration_ratio) ):
        vel_ratio = int(goal_request.velocity_ratio * 100)
        acc_ratio = int(goal_request.acceleration_ratio * 100)
        self.get_logger().info("Vel ratio: {0}".format(vel_ratio))
        self.get_logger().info("Acc ratio: {0}".format(acc_ratio))
        bot.set_point_to_point_common_params(vel_ratio, acc_ratio)
        bot.set_arc_params(vel_ratio, vel_ratio, acc_ratio, acc_ratio)
        bot.set_continous_trajectory_real_time_params(acc_ratio, vel_ratio, 10)
    else:
        self.get_logger().info('Wrong ratio in action goal field')
        return GoalResponse.REJECT

    # Wait until robot mode, e.g. PTP MOVEL TCP-coords, is set
    while not self.mode_ACK:
        pass

    self.get_logger().info('Received goal request')

    return GoalResponse.ACCEPT 
    
def is_goal_reached(target_pose, current_pose, threshold):
    for i in range(4):
        if abs(target_pose[i]-current_pose[i]) > threshold:
            return False
    return True

def is_pose_stable(pose_arr):
    if len(pose_arr) >= 2:
        if pose_arr[-1] == pose_arr[-2]:
            return True
    return False
    
def execute_callback_action(self, goal_handle, result, feedback_msg):
    # Execute a goal.
    self.get_logger().info('Executing goal...')

    start = time.time()
    # Start executing the action
    while not (is_goal_reached(self.target, self.dobot_pose, 0.5) and is_pose_stable(self.pose_arr)):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            bot.stop_queue(force=True) 
            bot.clear_queue()
            bot.start_queue()
            self.get_logger().info('Goal canceled')
            result.achieved_pose  = self.dobot_pose
            return


        # Update sequence
        feedback_msg.current_pose = self.dobot_pose
        self.pose_arr.append(self.dobot_pose)

        self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.current_pose))

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)

        # Sleep for demonstration purposes
        #time.sleep(0.1)

    end = time.time()
    length = end - start
    self.get_logger().info(f'It took {length} seconds!')

    goal_handle.succeed()

    result.achieved_pose  = self.dobot_pose

    self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

    return
    
def cancel_callback_action(self, goal_handle):
    # Request to cancel an action.
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT