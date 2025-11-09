while :
do
    ros2 action send_goal /PTP_action dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [150.68580627441406, 28.961393356323242, -0.5788955688476562, -79.12057495117188], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback
    ros2 action send_goal /PTP_action dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [222.115234375, 42.47715759277344, 8.731033325195312, -79.17351531982422], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback
done
