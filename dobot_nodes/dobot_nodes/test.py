from dobot_nodes.collision_detection_server import *

waypoints = PyBulletCollisionServer.arc_trajectory_to_discrete_waypoints([149.5470428466797, 5.013277530670166, 99.91801452636719], [275, 24, 99.918], [235, 46, 99.918])

for i in range(len(waypoints)):
    print(waypoints[i])