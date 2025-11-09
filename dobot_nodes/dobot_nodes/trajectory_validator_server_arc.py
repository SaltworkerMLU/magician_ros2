from dobot_msgs.srv import EvaluateARCTrajectory
import rclpy
from rclpy.node import Node
from dobot_nodes._dobot_kinematics import dobot_FK, dobot_IK
import math
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
import os.path as path
from geometry_msgs.msg import PoseStamped 
import tf_transformations
from dobot_nodes.collision_detection_server import PyBulletCollisionServer
import numpy as np
import os
from std_msgs.msg import Float64MultiArray




class PoseValidatorServiceArc(Node):

    def __init__(self):
        super().__init__('dobot_trajectory_validation_server_ARC')
        self.srv_Arc = self.create_service(EvaluateARCTrajectory, 'dobot_ARC_validation_service', self.PTP_trajectory_callback)
        self.subscription_TCP = self.create_subscription(Float64MultiArray, 'dobot_pose_raw', self.tcp_position_callback, 10)
        self.collision_server = PyBulletCollisionServer()


        # TCP pose before motion execution - in order to determine equation of linear trajectory
        self.dobot_pose = [] 

        self.trajectory_points = []
        self.max_velocity = 115 #TODO [mm/s]
        self.axis_1_range = {"min": -125, "max": 125}
        self.axis_2_range = {"min": -5, "max": 90}
        self.axis_3_range = {"min": -15, "max": 90} #"max": 60
        self.axis_4_range = {"min": -150, "max": 150}

        self.path_to_collision_model = None

        # User-defined axis limits

        # Axis 1
        self.declare_parameter('axis_1_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the first axis.", 
        additional_constraints = "The value must be between -125 and 125 degrees."))

        # Axis 2
        self.declare_parameter('axis_2_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the second axis.", 
        additional_constraints = "The value must be between -5 and 90 degrees."))

        # Axis 3
        self.declare_parameter('axis_3_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the third axis.", 
        additional_constraints = "The value must be between -15 and 80 degrees."))

        # Axis 4 
        self.declare_parameter('axis_4_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the fourth axis.", 
        additional_constraints = "The value must be between -150 and 150 degrees."))

        # Additional parameters used for collision detection



        self.add_on_set_parameters_callback(self.parameters_callback)


    def parameters_callback(self, params):

        for param in params:
            if param.name == 'axis_1_range':
                self.axis_1_range["min"] = param.value[0]
                self.axis_1_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_2_range':
                self.axis_2_range["min"] = param.value[0]
                self.axis_2_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_3_range':
                self.axis_3_range["min"] = param.value[0]
                self.axis_3_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_4_range':
                self.axis_4_range["min"] = param.value[0]
                self.axis_4_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            # elif param.name == "use_ground_collision_detection":
            #     if param.value == True:
            #         self.prevent_collision_with_ground = True
            #     elif param.value == False:
            #         self.prevent_collision_with_ground = False
            #     return SetParametersResult(successful=True)
            else:
                return SetParametersResult(successful=False)



    def tcp_position_callback(self, msg):
            self.dobot_pose = [float(msg.data[0])*1000, float(msg.data[1])*1000, float(msg.data[2])*1000, float(msg.data[3])]


    def PTP_trajectory_callback(self, request, response):
        (response.is_valid, response.message) = self.is_target_valid(request.p0,
                                                                     request.pc,
                                                                     request.pf)
        return response



    def are_angles_in_range_joint(self, angles):
        # Check DOCUMENTATION.md section "Dobot Magician Workspace" for details
        if (self.axis_1_range["min"] < angles[0] < self.axis_1_range["max"]) and \
           (self.axis_2_range["min"] < angles[1] < self.axis_2_range["max"]) and \
           (self.axis_3_range["min"] < angles[2] < self.axis_3_range["max"]) and \
           (self.axis_4_range["min"] < angles[3] < self.axis_4_range["max"]) and \
            not (90 > angles[2] > 65 and angles[1] < angles[2] - 70) and \
            not (35 > angles[2] > -5 and angles[1] > angles[2] + 55) and \
            not (90 > angles[1] > 40 and angles[2] < angles[1] - 55) and \
            not (20 > angles[1] > -15 and angles[2] > angles[1] + 70):
           return True
        return False
    
    def are_angles_in_range_cartesian(self, angles, position):
        JT1 = math.degrees(math.atan2(position[1],position[0]))
        JT2 = angles[1]
        if JT2 <= 40:
            offset = 0
        elif JT2 > 40:
            offset = JT2 - 40
        if (self.axis_1_range["min"] < angles[0] < self.axis_1_range["max"]) and \
           (self.axis_2_range["min"] < angles[1] < self.axis_2_range["max"]) and \
           (self.axis_3_range["min"] + offset < angles[2] < self.axis_3_range["max"]) and \
           (self.axis_4_range["min"] + JT1 < angles[3] < self.axis_4_range["max"] + JT1):
           return True
        return False

    def is_target_valid(self, p0, pc, pf):
        waypoints = self.collision_server.arc_trajectory_to_discrete_waypoints(p0, pc, pf)
        if waypoints == False:
            return (False, 'The 3 used points are collinear and/or too close to each other, hence no circle!')
        for point in waypoints:
            end_tool_rotation = pf.tolist()[3]
            np.append(point, end_tool_rotation)
            angles = dobot_IK(*point)
            if angles == False:
                return (False, 'Inv Kin solving error!')
            in_limit = self.are_angles_in_range_joint(angles)
            if in_limit == False:
                return (False, 'Joint limits violated')
        # is_trajectory_safe = self.collision_server.validate_trajectory(motion_type = target_type, current_pose = self.dobot_pose, target_point = target, detect_ground = self.prevent_collision_with_ground)
        # if is_trajectory_safe == False:
        #     return (False, 'A collision was detected during trajectory validation. The movement cannot be executed.')
        # else:
        return (True, 'Trajectory is safe and feasible.')


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PoseValidatorServiceArc()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()