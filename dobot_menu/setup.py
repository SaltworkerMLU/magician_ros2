from setuptools import setup
from glob import glob
import os

package_name = 'dobot_menu'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],

    # Creates the necessary installation directories
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/DobotControlPanel_layout.ui']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource/images', glob('resource/images/*.png')),
        ('share/' + package_name + '/resource/images', glob('resource/images/*.svg')),
        ('share/' + package_name + '/resource/commands', glob('resource/commands/*.bash'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='RQT-based GUI for Dobot Magician',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_menu = dobot_menu.main:main',
        ],
    },
)

