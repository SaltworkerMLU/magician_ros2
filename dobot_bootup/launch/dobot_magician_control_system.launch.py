import os, sys, subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.substitutions import LocalSubstitution, PythonExpression


def generate_launch_description():

    # -----------------------------------------------------------------------------------------------------------------
    # Check if the robot is physically connected

    shell_cmd = subprocess.Popen('lsusb | grep -E "Silicon Labs CP210x UART Bridge|QinHeng Electronics" ', shell=True, stdout=subprocess.PIPE)
    is_connected = shell_cmd.stdout.read().decode('utf-8')
    if not is_connected:
        sys.exit("Dobot is disconnected! Check if the USB cable and power adapter are plugged in.")
    # -----------------------------------------------------------------------------------------------------------------


    # -----------------------------------------------------------------------------------------------------------------
    # Check if MAGICIAN_TOOL env var has the right value 
    tool_env_var = str(os.environ.get('MAGICIAN_TOOL'))

    if tool_env_var == "None":
         sys.exit("MAGICIAN_TOOL env var is not set!")

    if tool_env_var not in ['none', 'pen', 'suction_cup', 'gripper', 'extended_gripper']:
         sys.exit("MAGICIAN_TOOL env var has an incorrect value!")

    valid_tool=["'", tool_env_var, "' == 'none'", 'or', \
                "'", tool_env_var, "' == 'pen'", 'or', \
                "'", tool_env_var, "' == 'suction_cup'", 'or', \
                "'", tool_env_var, "' == 'gripper'", 'or', \
                "'", tool_env_var, "' == 'extended_gripper'"
    ]
    # -----------------------------------------------------------------------------------------------------------------


    # -----------------------------------------------------------------------------------------------------------------
    # Nodes & launch files

    tool_null = Node(
        package='dobot_bootup',
        executable='set_tool_null',
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    alarms =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'alarms_analyzer.launch.py'
        ]],
        shell=True,
        output='log',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    gripper = Node(
        package='dobot_nodes',
        executable='gripper_server',
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    suction_cup = Node(
        package='dobot_nodes',
        executable='suction_cup_server',
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    homing =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'dobot_homing.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    auto_leveling =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'auto_leveling.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    trajectory_validator =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'dobot_validate_trajectory.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    PTP_action =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'dobot_PTP.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    Arc_action =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'dobot_Arc.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    draw_polygon =ExecuteProcess(
        cmd=[[
            'ros2 ', 'launch ', 'dobot_nodes ', 'draw_polygon.launch.py'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    robot_state = Node(
        package='dobot_nodes',
        executable='state_publisher',
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    dobot_menu = ExecuteProcess(
        cmd=[[
            'rqt ', '--force-discover ', '--clear-config ', '-s ', 'dobot_menu'
        ]],
        shell=True,
        output='screen',
        condition = IfCondition(PythonExpression(valid_tool))
    )

    # -----------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------
    # OnProcessStart events for information purposes 

    tool_null_event = RegisterEventHandler(
        OnProcessStart(
            target_action=tool_null,
            on_start=[
                LogInfo(msg='Loading tool parameters.')
            ]
        )
    )

    alarms_event = RegisterEventHandler(
        OnProcessStart(
            target_action=alarms,
            on_start=[
                LogInfo(msg='Starting the diagnostics module.')
            ]
        )
    )

    gripper_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gripper,
            on_start=[
                LogInfo(msg='Gripper control service started.')
            ]
        )
    )

    suction_cup_event = RegisterEventHandler(
        OnProcessStart(
            target_action=suction_cup,
            on_start=[
                LogInfo(msg='Suction Cup control service started.')
            ]
        )
    )

    homing_event = RegisterEventHandler(
        OnProcessStart(
            target_action=homing,
            on_start=[
                LogInfo(msg='Starting homing service.'),
                LogInfo(msg='Loading homing parameters.')
            ]
        )
    )

    auto_leveling_event = RegisterEventHandler(
        OnProcessStart(
            target_action=auto_leveling,
            on_start=[
                LogInfo(msg='Starting auto leveling service.'),
            ]
        )
    )

    trajectory_validator_event = RegisterEventHandler(
        OnProcessStart(
            target_action=trajectory_validator,
            on_start=[
                LogInfo(msg='Strating trajectory validator service.'),
                LogInfo(msg='Loading kinematics parameters.')
            ]
        )
    )

    PTP_action_event = RegisterEventHandler(
        OnProcessStart(
            target_action=PTP_action,
            on_start=[
                LogInfo(msg='Starting PointToPoint action server.'),
                LogInfo(msg='Setting speed and acceleration values.')
            ]
        )
    )

    Arc_action_event = RegisterEventHandler(
        OnProcessStart(
            target_action=Arc_action,
            on_start=[
                LogInfo(msg='Starting Arc action server.'),
                LogInfo(msg='Setting speed and acceleration values.')
            ]
        )
    )

    draw_polygon_event = RegisterEventHandler(
        OnProcessStart(
            target_action=draw_polygon,
            on_start=[
                LogInfo(msg='Starting draw_polygon action server.'),
            ]
        )
    )

    robot_state_event = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state,
            on_start=[
                LogInfo(msg='Dobot state updater node started.'),
                LogInfo(msg='Dobot Magician control stack has been launched correctly')
            ]
        )
    )

    dobot_menu_event = RegisterEventHandler(
        OnProcessStart(
            target_action=dobot_menu,
            on_start=[
                LogInfo(msg='Booting up dobot menu.'),
            ]
        )
    )
    # -----------------------------------------------------------------------------------------------------------------



    # -----------------------------------------------------------------------------------------------------------------
    # Shutdown event handler
    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Dobot Magician control system launch was asked to shutdown: ',
                    LocalSubstitution('event.reason')]
            )]
        )
    )
    # -----------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------
    # Launch scheduler


    tool_null_sched = TimerAction(
        period=1.0,
        actions=[tool_null]
        )

    alarms_sched = TimerAction(
        period=2.0,
        actions=[alarms]
        )

    gripper_sched = TimerAction(
        period=2.0,
        actions=[gripper]
        )

    suction_cup_sched = TimerAction(
        period=2.0,
        actions=[suction_cup]
        )

    homing_sched = TimerAction(
        period=3.0,
        actions=[homing]
        )
 
    auto_leveling_sched = TimerAction(
        period=3.0,
        actions=[auto_leveling]
        )

    trajectory_validator_sched = TimerAction(
        period=5.0,
        actions=[trajectory_validator]
        )

    PTP_action_sched = TimerAction(
        period=7.0,
        actions=[PTP_action]
        )
    
    Arc_action_sched = TimerAction(
        period=7.0,
        actions=[Arc_action]
        )
    
    draw_polygon_sched = TimerAction(
        period=15.0,
        actions=[draw_polygon]
        )

    robot_state_sched = TimerAction(
        period=18.0,
        actions=[robot_state]
        )
    
    dobot_menu_sched = TimerAction(
        period=18.0,
        actions=[dobot_menu]
        )

    # -----------------------------------------------------------------------------------------------------------------


    return LaunchDescription([
        tool_null_event,
        alarms_event,
        gripper_event,
        suction_cup_event,
        homing_event,
        #auto_leveling_event,
        trajectory_validator_event,
        PTP_action_event,
        Arc_action_event,
        draw_polygon_event,
        robot_state_event,
        dobot_menu_event,
        tool_null_sched,
        alarms_sched,
        gripper_sched,
        suction_cup_sched,
        homing_sched,
        #auto_leveling_sched,
        trajectory_validator_sched,
        PTP_action_sched,
        Arc_action_sched,
        draw_polygon_sched,
        robot_state_sched,
        dobot_menu_sched,
        on_shutdown
    ])
