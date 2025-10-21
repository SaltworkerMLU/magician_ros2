from setuptools import setup
import os
from glob import glob

package_name = 'dobot_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Sample scripts to test basic functionalities',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_arc = dobot_demos.Arc_client:main',
            'test_auto_leveling = dobot_demos.auto_leveling_client:main',
            'test_draw_circle = dobot_demos.draw_circle:main',
            'test_gripper = dobot_demos.gripper_client:main',
            'test_homing = dobot_demos.homing_client:main',
            'test_point_to_point = dobot_demos.PTP_client:main',
            'test_suction_cup = dobot_demos.suction_cup_client:main',
            'test_pick_and_place = dobot_demos.pick_and_place:main',
        ],
    },
)

