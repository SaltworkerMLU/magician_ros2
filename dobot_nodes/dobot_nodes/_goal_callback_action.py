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
    else:
        self.get_logger().info('Wrong ratio in action goal field')
        return GoalResponse.REJECT
    
    # Runs only if DobotPTPServer is used
    try:
        self.target = goal_request.target_pose
        self.motion_type = goal_request.motion_type

        # Check if trajectory is feasible within robot workspace
        validation_response = self.send_request_check_trajectory(self.target, self.motion_type)
        if validation_response.is_valid == False:
            self.get_logger().warn("Goal rejected: {0}".format(validation_response))
            return GoalResponse.REJECT
        
        self.get_logger().info('Goal: {0}'.format(self.target))
        self.get_logger().info('Mode: {0}'.format(self.motion_type))
    except:
        pass
    
    # Runs only if DobotArcServer is used
    try:
        self.circumference_point = goal_request.circumference_point
        self.ending_point = goal_request.ending_point
        
        # Check if trajectory is feasible within robot workspace
        validation_response1 = self.send_request_check_trajectory(self.circumference_point)
        if validation_response1.is_valid == False:
            self.get_logger().warn("Circumference point rejected: {0}".format(validation_response1))
            return GoalResponse.REJECT
        
        validation_response = self.send_request_check_trajectory(self.ending_point)
        if validation_response.is_valid == False:
            self.get_logger().warn("Ending point rejected: {0}".format(validation_response))
            return GoalResponse.REJECT

        self.get_logger().info('Checkpoint: {0}'.format(self.circumference_point))
        self.get_logger().info('Goal: {0}'.format(self.ending_point))
    except:
        pass

    self.get_logger().info("Result of calling validation service: is valid? {0}, description: {1}".format(validation_response.is_valid, validation_response.message))

    # Wait until robot mode, e.g. PTP MOVEL TCP-coords, is set
    while not self.mode_ACK:
        pass

    self.get_logger().info('Received goal request')

    # Try only if DobotPTPServer is used
    try:
        # Check if motion type is a valid one
        if self.motion_type in self.motion_types_list:
            return GoalResponse.ACCEPT 
        else:
            self.get_logger().info('The motion mode you specified does not exist!')
            return GoalResponse.REJECT
    except:
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
    
async def execute_callback_action(self, goal_handle):
    # Execute a goal.
    self.get_logger().info('Executing goal...')

    # Runs only if DobotPTPServer is used
    try:
        bot.set_point_to_point_command(self.motion_type, self.target[0], self.target[1], self.target[2], self.target[3])
        feedback_msg = PointToPoint.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]
        self.pose_arr = []

        result = PointToPoint.Result()
    except:
        pass

    # Runs only if DobotArcServer is used
    try:
        bot.set_arc_command([self.circumference_point[0], 
                             self.circumference_point[1],
                             self.circumference_point[2],
                             self.circumference_point[3]], 
                            [self.ending_point[0], 
                             self.ending_point[1], 
                             self.ending_point[2], 
                             self.ending_point[3]])
        
        feedback_msg = ArcMotion.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]
        self.pose_arr = []

        result = ArcMotion.Result()

        self.target = self.ending_point
    except:
        pass

    # Start executing the action
    while not (is_goal_reached(self.target, self.dobot_pose, 0.2) and is_pose_stable(self.pose_arr)):
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

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)

        # Sleep for demonstration purposes
        time.sleep(0.1)


    goal_handle.succeed()

    result.achieved_pose  = self.dobot_pose

    self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

    return result
    
def cancel_callback_action(self, goal_handle):
    # Accept or reject a client request to cancel an action.
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT