import time
import numpy as np
import pybullet as pyb
import pybullet_data
import os.path as path
import sys
from math import radians
from dobot_nodes._collision_utils import NamedCollisionObject, CollisionDetector
import math
from dobot_nodes._dobot_inv_kin import calc_inv_kin
import os

global model_config


class PyBulletCollisionServer():


    @staticmethod
    def load_environment(client_id):

        pyb.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=client_id
        )

        model_prefix = path.abspath(path.join(__file__ ,"../../..")) 
        model_prefix = model_prefix.split("dobot_kinematics",1)[0]+ "dobot_description/share/dobot_description/meshes/collision/urdf_models/"

        tool_mounted = os.environ.get('MAGICIAN_TOOL')

        if tool_mounted == "none":
            path_to_collision_model = model_prefix + "3DOF.urdf"
        elif tool_mounted == "pen":
            path_to_collision_model = model_prefix + "pen.urdf"
        elif tool_mounted == "suction_cup":
            path_to_collision_model = model_prefix + "suction_cup.urdf"
        elif tool_mounted == "gripper":
            path_to_collision_model = model_prefix + "gripper.urdf"
        elif tool_mounted == "extended_gripper":
            path_to_collision_model = model_prefix + "extended_gripper.urdf"


        global model_config
        prefix, model_config = os.path.split(path_to_collision_model)

        dobot_magician_id = pyb.loadURDF(
            path_to_collision_model,
            [0, 0, 0],
            useFixedBase=True,
            physicsClientId=client_id,
        )

        ground_id = pyb.loadURDF(
            "plane.urdf", [0, 0, -0.131305], useFixedBase=True, physicsClientId=client_id
        )

        bodies = {
            "robot": dobot_magician_id,
            "ground": ground_id,
        }

        for id in bodies.values():
            if id <= -1 :
                print("Could not load URDF model")
                sys.exit(1)

        return bodies


    @staticmethod
    def set_robot_configuration(angles):
        """Set robot configuration."""
        q = np.zeros(4)
        q[0] = radians(angles[0])
        q[1] = radians(angles[1])
        q[2] = radians(angles[2]) - q[1]
        q[3] = -(q[1]+q[2])
        return q

    @staticmethod
    def update_robot_configuration(robot_id, q, gui_id):
        """Set the robot configuration."""
        for i in range(4):
            pyb.resetJointState(
                robot_id, i+1, q[i], physicsClientId = gui_id
            )


    @staticmethod
    def linear_trajecory_to_discrete_waypoints(start, target, step_len = 0.5):
        waypoints = []

        x, y, z = [start[0], target[0]], [start[1], target[1]], [start[2], target[2]]

        steps_num = 1
        while True:
            dist_now = math.dist([x[0], y[0], z[0]], [x[0] + (x[1]-x[0])*(1/steps_num), y[0] + (y[1]-y[0])*(1/steps_num), z[0] + (z[1]-z[0])*(1/steps_num)])
            dist_next = math.dist([x[0], y[0], z[0]], [x[0] + (x[1]-x[0])*(1/(steps_num+1)), y[0] + (y[1]-y[0])*(1/(steps_num+1)), z[0] + (z[1]-z[0])*(1/(steps_num+1))])

            if dist_now > step_len and dist_next < step_len:
                steps_num = steps_num + 3
                break
            else:
                steps_num = steps_num + 1

        for t in range(steps_num-1):
            t =t / (steps_num-2)
            waypoints.append([x[0] + (x[1]-x[0])*t, y[0] + (y[1]-y[0])*t, z[0] + (z[1]-z[0])*t])

        return waypoints
    
    @staticmethod
    def P_t(t, P_center, r, u, w):
        return P_center + r * (u * math.cos(t) + w * math.sin(t))

    @staticmethod
    def arc_trajectory_to_discrete_waypoints(P0, Pc, Pf, step_len = 1):
        P0 = np.array(P0[0:3])
        Pc = np.array(Pc[0:3])
        Pf = np.array(Pf[0:3])

        # Calculate the circle of the 3 points
        vc = Pc - P0
        vf = Pf - P0

        N = np.cross(vc,vf)

        if np.linalg.norm(N) == 0:
            return False

        P_center = P0 + ( ( pow(np.linalg.norm(vf),2) * np.cross(N,vc) + 
                            pow(np.linalg.norm(vc),2) * np.cross(vf,N)) / 
                            (2 * pow(np.linalg.norm(N), 2)) ) 

        r = np.linalg.norm(P_center - P0)

        # If the radius of the produced circle is too small, the Dobot Crashes
        if r < 0.2:
            return False

        u = (P0 - P_center) / r
        w = np.cross(N / np.linalg.norm(N), u)
        
        # Calculate span of circle where the trajectory will be done
        angle = math.acos(np.dot(u, (Pf-P_center)) / 
                          (np.linalg.norm(u) * np.linalg.norm(Pf-P_center)))

        # Check if any vector element of P(t) and Pf do not match
        if ( np.round( PyBulletCollisionServer.P_t(angle, P_center, r, u, w),2 ) != np.round(Pf,2) ).any():
            angle = 2 * math.pi - angle

        waypoints = []
        for t in range(0,int(math.degrees(angle)), step_len):
            next_point = PyBulletCollisionServer.P_t(math.radians(t), P_center, r, u, w)
            waypoints.append(next_point)

        # Add Pf to waypoints
        waypoints.append(PyBulletCollisionServer.P_t(angle, P_center, r, u, w))

        # Add maximum points to waypoint
        
        return waypoints

        """waypoints = []

        x = [P0[0], Pc[0], Pf[0]]
        y = [P0[1], Pc[1], Pf[1]]
        z = [P0[2], Pc[2], Pf[2]]

        # Form vectors between the points
        v1 = Pc - P0
        v2 = Pf - P0

        # Calculate the normal vector
        n = np.cross(v1, v2)

        # This only happens if the 3 dots are collinear (lying in the same straight line)
        # In this case, no circle can be formed
        if np.linalg.norm(n) == 0:
            return False

        # Normalize the normal vector (make its length equal 1)
        n_norm = n / np.linalg.norm(n)
        
        # Some expressions
        v11 = np.dot(v1, v1)
        v22 = np.dot(v2, v2)
        v12 = np.dot(v1, v2)
        b = (v11 * v22 - pow(v12, 2))
        k1 = 0.5 * (np.dot(v11, v22) - np.dot(v22, v12)) / b
        k2 = 0.5 * (np.dot(v11, v22) - np.dot(v11, v12)) / b

        # Find the 3D circle center
        P_center = P0 + k1 * v1 + k2 * v2

        # Find the 3D circle radius
        radius = np.linalg.norm(P0 - P_center)

        # Define two othogonal unit vectors
        u = (P0 - P_center) / radius
        w = np.cross(n_norm, u)"""


    
    def validate_trajectory(self, motion_type, current_pose, target_point, detect_ground):
        # MOTION_TYPE_MOVJ_XYZ 1
        # MOTION_TYPE_MOVL_XYZ 2
        # MOTION_TYPE_MOVJ_ANGLE 4
        # MOTION_TYPE_MOVL_ANGLE 5
        col_id = pyb.connect(pyb.DIRECT, options="--opengl3")

        if col_id == -1:
            print("Not connected")
            sys.exit(1)


        collision_bodies = PyBulletCollisionServer.load_environment(col_id)

        robot_id = collision_bodies["robot"]

        end_effector_link = NamedCollisionObject("robot", "magician_link_4") 
        if model_config != "3DOF.urdf":
            gripper = NamedCollisionObject("robot", "end_effector_part")  
        base = NamedCollisionObject("robot", "magician_base_link")
        link1 = NamedCollisionObject("robot", "magician_link_1")
        ground = NamedCollisionObject("ground")


        if detect_ground == True and model_config=="3DOF.urdf":
            col_detector = CollisionDetector(
                col_id,
                collision_bodies,
                [(end_effector_link, base), (end_effector_link, ground), (end_effector_link, link1)],
            )
        elif detect_ground == False and model_config=="3DOF.urdf":
            col_detector = CollisionDetector(
                col_id,
                collision_bodies,
                [(end_effector_link, base), (end_effector_link, link1)],
            )
        elif detect_ground == True and model_config!="3DOF.urdf":
            col_detector = CollisionDetector(
                col_id,
                collision_bodies,
                [(end_effector_link, base), (end_effector_link, ground), (end_effector_link, link1), (gripper, base), (gripper, ground), (gripper, link1)],
            )
        elif detect_ground == False and model_config!="3DOF.urdf":
            col_detector = CollisionDetector(
                col_id,
                collision_bodies,
                [(end_effector_link, base), (end_effector_link, link1), (gripper, base), (gripper, link1)],
            )
        

        if motion_type == 1 or motion_type == 4:
            if motion_type == 1:
                angles = calc_inv_kin(*target_point)
                q = PyBulletCollisionServer.set_robot_configuration(angles[0:3])
            elif motion_type == 4:
                q = PyBulletCollisionServer.set_robot_configuration(target_point)
            PyBulletCollisionServer.update_robot_configuration(robot_id, q, gui_id=col_id)
            pyb.stepSimulation(physicsClientId=col_id)

            in_col = col_detector.in_collision(
                q, margin = 0.0
            )
            if in_col:
                return False

            return True

        if motion_type == 2 or motion_type == 5:
            waypoints = PyBulletCollisionServer.linear_trajecory_to_discrete_waypoints(current_pose, target_point)
            for point in waypoints:
                if motion_type == 2:
                    end_tool_rotation = target_point.tolist()[3]
                elif motion_type == 5:
                    end_tool_rotation = target_point[3]
                point.append(end_tool_rotation)
                angles = calc_inv_kin(*point)
                q = PyBulletCollisionServer.set_robot_configuration(angles[0:3])
                PyBulletCollisionServer.update_robot_configuration(robot_id, q, gui_id=col_id)
                pyb.stepSimulation(physicsClientId=col_id)
                
                in_col = col_detector.in_collision(
                    q, margin = 0.0
                )
                if in_col:
                    return False


            return True
