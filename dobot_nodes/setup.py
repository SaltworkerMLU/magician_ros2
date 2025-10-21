from setuptools import setup
from glob import glob
import os

package_name = 'dobot_nodes'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mathias Lykholt - Ustrup',
    maintainer_email='mathiaslyus@gmail.com',
    description='ROS 2 nodes enabling Dobot Magician functionalities using ROS 2 topics, services, and actions.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arc_server = dobot_nodes.arc_server:main',
            'draw_circle_action = dobot_nodes.draw_circle_action:main',
            'alarms_parser = dobot_nodes.alarms_parser:main',
            'state_publisher = dobot_nodes.dobot_state_publ:main',
            'gripper_server = dobot_nodes.gripper_server:main',
            'suction_cup_server = dobot_nodes.suction_cup_server:main',  
            'homing_server = dobot_nodes.homing_server:main',
            'auto_leveling_server = dobot_nodes.auto_leveling:main',
            'trajectory_validator_client = dobot_nodes.trajectory_validator_client:main',
            'trajectory_validator_server = dobot_nodes.trajectory_validator_server:main',
            'PTP_server = dobot_nodes.PTP_server:main',
            'sliding_rail_server = dobot_nodes.sliding_rail_server:main',
        ],
    },
)
