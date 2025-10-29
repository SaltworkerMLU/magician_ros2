#!/usr/bin/env python

from __future__ import division

import itertools
import os
from std_msgs.msg import String
from dobot_driver.dobot_handle import bot

from ament_index_python import get_resource
from python_qt_binding import loadUi
from rqt_py_common.message_helpers import get_message_class
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import subprocess
import rclpy
from geometry_msgs.msg import PoseStamped 
import math
import tf_transformations
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from dobot_msgs.msg import GripperStatus
import sys
from time import sleep




class DobotMenu(QWidget):

    def __init__(self, node, plugin=None):

        super(DobotMenu, self).__init__() # Call any existing __init__ methods using super()

        # ------------------------------------------------- #
        # Check if dobot magician is connected to Linux     #
        # ------------------------------------------------- #
        self.shell_cmd = subprocess.Popen('lsusb | grep -E "Silicon Labs CP210x UART Bridge|QinHeng Electronics" ', shell=True, stdout=subprocess.PIPE)
        self.is_connected = self.shell_cmd.stdout.read().decode('utf-8') # Check if dobot is connected
        
        # ---------------- #
        # Load the UI file #
        # ---------------- #
        _, package_path = get_resource('packages', 'dobot_menu') # Get repository path
        ui_file = os.path.join(package_path, 'share', 'dobot_menu', 'resource', 'DobotControlPanel_layout.ui')
        loadUi(ui_file, self)

        # ------------------------------------------------------------------------- #
        # Initialize QWidget properties                                             #
        # ------------------------------------------------------------------------- #
        self._node = node               # Store rclpy.node.Node object
        self._logText = ''              # Used to store log text
        self.process  = QProcess(self)  # Used to run shell commands
        self._plugin = plugin           # NOTE Not used, but could be useful to store a reference to the parent plugin

        # Timer for updating the line position
        self._line_update_timer = QTimer(self)  # Initialize Qt timer
        self._line_update_timer.timeout.connect(self._on_line_update_timer)
        self._line_update_timer.start(50)       # Start timer at 50ms intervals

        self.dobotJoints = None                   # Store current joint states
        self.dobotPose = []                     # Store current TCP pose
        self.frame = "base"                     # Store current frame mode ("base" or "joint")
        self.valueType = "base"                 # Store current programming value type
        self.movementType = "movej"             # Store current programming movement type
        self.gripperType = "gripper"            # Store grupper type (gripper, suction cup)
        self.gripperClose = False               # Store current gripper mode (on or off)
        self.suctionCupOn = False               # Store current suction cup mode (on or off)
        self.import_path = os.path.join(package_path, 'share', 'dobot_menu', 'resource', 'commands') # Default import path

        # ----------------------------------- #
        # Create subscriptions and publishers #
        # ----------------------------------- #
        self.subscription_TCP = self._node.create_subscription(
            Float64MultiArray,
            'dobot_pose_raw',
            self.tcp_position_callback,
            10)

        self.subscription_joints = self._node.create_subscription(
            JointState,
            'dobot_joint_states',
            self.joints_positions_callback,
            10)

        self.gripper_state_publ = self._node.create_publisher(GripperStatus, 'gripper_status_rviz', 10)

        # ------------------------------------------------- #
        # Initialize UI element properties and callbacks    #
        # ------------------------------------------------- #

        # Control Tab
        self.JT1Plus.pressed.connect(lambda:self.JT1_move(self.JT1Plus))
        self.JT1Plus.released.connect(self.JT_IDLE)
        self.JT1Minus.pressed.connect(lambda:self.JT1_move(self.JT1Minus))
        self.JT1Minus.released.connect(self.JT_IDLE)
        
        self.JT2Plus.pressed.connect(lambda:self.JT2_move(self.JT2Plus))
        self.JT2Plus.released.connect(self.JT_IDLE)
        self.JT2Minus.pressed.connect(lambda:self.JT2_move(self.JT2Minus))
        self.JT2Minus.released.connect(self.JT_IDLE)

        self.JT3Plus.pressed.connect(lambda:self.JT3_move(self.JT3Plus))
        self.JT3Plus.released.connect(self.JT_IDLE)
        self.JT3Minus.pressed.connect(lambda:self.JT3_move(self.JT3Minus))
        self.JT3Minus.released.connect(self.JT_IDLE)

        self.JT4Plus.pressed.connect(lambda:self.JT4_move(self.JT4Plus))
        self.JT4Plus.released.connect(self.JT_IDLE)
        self.JT4Minus.pressed.connect(lambda:self.JT4_move(self.JT4Minus))
        self.JT4Minus.released.connect(self.JT_IDLE)

        self.HomingButton.clicked.connect(self.button_clicked_HomingButton)
        self.EStopButton.clicked.connect(self.button_clicked_EStopButton)

        # Frame selection - defaults to base frame
        self.BaseFrame.setChecked(True)
        self.BaseFrame.toggled.connect(lambda:self.framestate_control_tab(self.BaseFrame))
        self.JointFrame.toggled.connect(lambda:self.framestate_control_tab(self.JointFrame))

        # ------------------- #
        # Speed Tab Velocity  #
        # ------------------- #
        self.Joint1VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT1Vel))
        self.Joint2VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT2Vel))
        self.Joint3VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT3Vel))
        self.Joint4VelSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT4Vel))

        self.XVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.XVel))
        self.YVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.YVel))
        self.ZVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.ZVel))
        self.RVelSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.RVel))

        # -----------------------------------------------------------------------------

        self.Joint1VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint1VelSlider))
        self.Joint2VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint2VelSlider))
        self.Joint3VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint3VelSlider))
        self.Joint4VelSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint4VelSlider))

        self.XVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.XVelSlider))
        self.YVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.YVelSlider))
        self.ZVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.ZVelSlider))
        self.RVelSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.RVelSlider))

        # ----------------------- #
        # Speed Tab Acceleration  #
        # ----------------------- #
        self.Joint1AccSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT1Acc))
        self.Joint2AccSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT2Acc))
        self.Joint3AccSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT3Acc))
        self.Joint4AccSlider.valueChanged.connect(lambda:self.valuechange_joints(self.JT4Acc))

        self.XAccSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.XAcc))
        self.YAccSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.YAcc))
        self.ZAccSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.ZAcc))
        self.RAccSlider.valueChanged.connect(lambda:self.valuechange_cartesian(self.RAcc))

        # -----------------------------------------------------------------------------

        self.Joint1AccSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint1AccSlider))
        self.Joint2AccSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint2AccSlider))
        self.Joint3AccSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint3AccSlider))
        self.Joint4AccSlider.sliderReleased.connect(lambda:self.change_vel_joints(self.Joint4AccSlider))

        self.XAccSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.XAccSlider))
        self.YAccSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.YAccSlider))
        self.ZAccSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.ZAccSlider))
        self.RAccSlider.sliderReleased.connect(lambda:self.change_vel_cartesian(self.RAccSlider))

        # -----------------------------------------------------------------------------
        
        # initial setup (REMEMBER ACCELERATION)
        if self.is_connected:
            bot.set_jog_common_params(100, 1) # (vel_ratio, acc_ratio)
            bot.set_jog_joint_params([100, 100, 100, 100], [100, 100, 100, 100])
            bot.set_jog_coordinate_params([100, 100, 100, 100], [100, 100, 100, 100])

            # Start homing the robot
            """command = 'ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure'
            subprocess.Popen(
                        command, universal_newlines=True, shell=True,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()"""

        # Programming Tab
        self.programmingButtonTeachEE.clicked.connect(self.EE_teach_command)

        self.EEGripperClose.setChecked(True)
        self.EEGripperClose.setAutoExclusive(False)
        self.EEGripperOpen.setAutoExclusive(False)
        self.EESuctionCupOn.setAutoExclusive(False)
        self.EESuctionCupOff.setAutoExclusive(False)

        self.EESuctionCupOn.toggled.connect(lambda:self.suctionCup_control_tab(self.EESuctionCupOn))
        self.EESuctionCupOff.toggled.connect(lambda:self.suctionCup_control_tab(self.EESuctionCupOff))
        self.EEGripperClose.toggled.connect(lambda:self.gripper_control_tab(self.EEGripperClose))
        self.EEGripperOpen.toggled.connect(lambda:self.gripper_control_tab(self.EEGripperOpen))

        self.programmingButtonTeachPTP.clicked.connect(self.PTP_teach_command)

        self.programmingButtonExecute.clicked.connect(self.execute_command)
        self.programmingButtonExport.clicked.connect(self.export_command)
        self.programmingButtonImport.clicked.connect(self.import_command)

        self.programmingLineEditExportPath.setText("/mnt/c/users")

        self.programmingLineEditCode.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.programmingLineEditCode.textChanged.connect(lambda:self.execute_save_command(self.programmingLineEditCode))         # Autosave the code when the text is changed
        self.programmingLineEditCommandPath.textChanged.connect(lambda:self.execute_load_command(self.programmingLineEditCode))  # Load the code when the command file name is changed
        
        self.execute_load_command(self.programmingLineEditCode) # Load the default code to file

        # Frame selection - defaults to MoveJ and Base Values
        self.MoveJCommand.setChecked(True)
        self.BaseValues.setChecked(True)
        self.JointValues.toggled.connect(lambda:self.framestate_programming_tab_valueType(self.JointValues))
        self.BaseValues.toggled.connect(lambda:self.framestate_programming_tab_valueType(self.BaseValues))
        self.MoveJCommand.toggled.connect(lambda:self.framestate_programming_tab_movement(self.MoveJCommand))
        self.MoveLCommand.toggled.connect(lambda:self.framestate_programming_tab_movement(self.MoveLCommand))


        # Drawing tab
        self.DrawingButtonTeachArc.clicked.connect(self.Arc_teach_command)
        self.DrawXCoordinateSliderCircumference.valueChanged.connect(lambda:self.change_XY_dot(self.DrawXCoordinateSliderCircumference))
        self.DrawYCoordinateSliderCircumference.valueChanged.connect(lambda:self.change_XY_dot(self.DrawYCoordinateSliderCircumference))

        self.DrawXCoordinateSliderEnd.valueChanged.connect(lambda:self.change_XY_dot(self.DrawXCoordinateSliderEnd))
        self.DrawYCoordinateSliderEnd.valueChanged.connect(lambda:self.change_XY_dot(self.DrawYCoordinateSliderEnd))

        self.DrawTeachZCoordinateButtonCircumference.clicked.connect(self.teachZCoordinateCircumference)
        self.DrawTeachZCoordinateButtonEnd.clicked.connect(self.teachZCoordinateEnd)

        # ------------------------------------------------- #
        # --- QGraphicsView/Scene setup for 2D grid tab --- #
        # ------------------------------------------------- #
        self.sceneSide = QGraphicsScene()
        self.sceneTop = QGraphicsScene()
        self.dobotViewSide.setScene(self.sceneSide)
        self.dobotViewTop.setScene(self.sceneTop)

        # Set background image for the QGraphicsScene and scale it to the scene/view size
        bg_image_path = os.path.join(package_path, 'share', 'dobot_menu', 'resource', 'images', 'dobot_magician_2D_side.png')
        bg_pixmap = QPixmap(bg_image_path)
        top_image_path = os.path.join(package_path, 'share', 'dobot_menu', 'resource', 'images', 'dobot_magician_2D_top.png')
        top_pixmap = QPixmap(top_image_path)

        # Get the size of the graphicsViewSide widget
        view_size = self.dobotViewSide.size()
        scaled_pixmap = bg_pixmap.scaled(view_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        self.sceneSide.setSceneRect(0, 0, view_size.width()-10, view_size.height()-10)
        self.sceneSide.setBackgroundBrush(QBrush(scaled_pixmap))

        # Get the size of the dobotViewTop widget
        view_size = self.dobotViewTop.size()
        top_scaled_pixmap = top_pixmap.scaled(view_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        self.sceneTop.setSceneRect(0, 0, view_size.width()-10, view_size.height()-10)
        self.sceneTop.setBackgroundBrush(QBrush(top_scaled_pixmap))

        # Draw initial line of side view
        self.link_2_start_side = [105, 165]
        self.link_2_start_top = [150, 150]
        self.sceneScaling = 0.4
        self.EE_XY_len = 60 * self.sceneScaling # mm
        self.link_len = [0, 135 * self.sceneScaling, 147 * self.sceneScaling]

        self.link_2_side = QGraphicsLineItem(self.link_2_start_side[0], self.link_2_start_side[1], 
                                             self.link_2_start_side[0], self.link_2_start_side[1] - self.link_len[1])
        self.link_3_side = QGraphicsLineItem(self.link_2_start_side[0], self.link_2_start_side[1] - self.link_len[1], 
                                             self.link_2_start_side[0] + self.link_len[2], self.link_2_start_side[1] - self.link_len[1])
        self.link_2_side.setPen(QPen(Qt.black, 3))
        self.link_3_side.setPen(QPen(Qt.gray, 3))
        self.sceneSide.addItem(self.link_2_side)
        self.sceneSide.addItem(self.link_3_side)

        # Draw initial line of top view
        self.link_2_top = QGraphicsLineItem(self.link_2_start_top[0], self.link_2_start_top[1], 
                                            self.link_2_start_top[0], self.link_2_start_top[1])
        self.link_3_top = QGraphicsLineItem(self.link_2_start_top[0], self.link_2_start_top[1], 
                                            self.link_2_start_top[0] + self.link_len[2], self.link_2_start_top[1])
        self.link_2_top.setPen(QPen(Qt.black, 3))
        self.link_3_top.setPen(QPen(Qt.gray, 3))
        self.sceneTop.addItem(self.link_2_top)
        self.sceneTop.addItem(self.link_3_top)

        # Add dots 
        self.EEDrawDot = QGraphicsEllipseItem(self.link_2_start_top[0]-2, self.link_2_start_top[1]-2, 4, 4)
        self.EEDrawDot.setPen(QPen(Qt.red))
        self.EEDrawDot.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        self.sceneTop.addItem(self.EEDrawDot)

        self.CDot = QGraphicsEllipseItem(self.link_2_start_top[0]-2, self.link_2_start_top[1]-2, 4, 4)
        self.CDot.setPen(QPen(Qt.green))
        self.CDot.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        self.sceneTop.addItem(self.CDot)

        self.EndDot = QGraphicsEllipseItem(self.link_2_start_top[0]-2, self.link_2_start_top[1]-2, 4, 4)
        self.EndDot.setPen(QPen(Qt.darkRed))
        self.EndDot.setBrush(QBrush(Qt.darkRed, Qt.SolidPattern))
        self.sceneTop.addItem(self.EndDot)


    def _on_line_update_timer(self):
        if self.dobotJoints is not None:
            self.update_line_position()

    def change_XY_dot(self, field):
        if field.objectName() == "DrawXCoordinateSliderCircumference":
            self.DrawXCoordinateAxisCircumference.setText(str(self.DrawXCoordinateSliderCircumference.value()))
        if field.objectName() == "DrawYCoordinateSliderCircumference":
            self.DrawYCoordinateAxisCircumference.setText(str(self.DrawYCoordinateSliderCircumference.value()))

        if field.objectName() == "DrawXCoordinateSliderEnd":
            self.DrawXCoordinateAxisEnd.setText(str(self.DrawXCoordinateSliderEnd.value()))
        if field.objectName() == "DrawYCoordinateSliderEnd":
            self.DrawYCoordinateAxisEnd.setText(str(self.DrawYCoordinateSliderEnd.value()))

    def teachZCoordinateCircumference(self):
        self.DrawZCoordinateAxisCircumference.setText(str(round(self.dobotPose[2], 3)))
    
    def teachZCoordinateEnd(self):
        self.DrawZCoordinateAxisEnd.setText(str(round(self.dobotPose[2], 3)))

    def update_line_position(self):
        #self._node.get_logger().info(f"Updating line position with joint states: {self.dobotJoints}")
        _link2_rotated_by_joint = [self.link_len[1] * math.sin(self.dobotJoints[1]), 
                                   self.link_len[1] * math.cos(self.dobotJoints[1])]
        
        _link3_rotated_by_joint = [self.link_len[2]*math.cos(self.dobotJoints[2]), 
                                   self.link_len[2]*math.sin(self.dobotJoints[2])]
        
        self.link_2_side.setLine(self.link_2_start_side[0], 
                                 self.link_2_start_side[1], 
                                 self.link_2_start_side[0] + _link2_rotated_by_joint[0],    
                                 self.link_2_start_side[1] - _link2_rotated_by_joint[1]
                                 )
        self.link_3_side.setLine(self.link_2_start_side[0] + _link2_rotated_by_joint[0], 
                                 self.link_2_start_side[1] - _link2_rotated_by_joint[1],
                                 self.link_2_start_side[0] + _link2_rotated_by_joint[0] + _link3_rotated_by_joint[0],
                                 self.link_2_start_side[1] - _link2_rotated_by_joint[1] + _link3_rotated_by_joint[1]
                                 )
        
        self.link_2_top.setLine(self.link_2_start_top[0], 
                                self.link_2_start_top[1], 
                                self.link_2_start_top[0] + math.cos(self.dobotJoints[0]) * _link2_rotated_by_joint[0], 
                                self.link_2_start_top[1] - math.sin(self.dobotJoints[0]) * _link2_rotated_by_joint[0],
                                )
        self.link_3_top.setLine(self.link_2_start_top[0] + math.cos(self.dobotJoints[0]) * _link2_rotated_by_joint[0], 
                                self.link_2_start_top[1] - math.sin(self.dobotJoints[0]) * _link2_rotated_by_joint[0],
                                self.link_2_start_top[0] + math.cos(self.dobotJoints[0]) * ( _link2_rotated_by_joint[0] + _link3_rotated_by_joint[0] ), 
                                self.link_2_start_top[1] - math.sin(self.dobotJoints[0]) * ( _link2_rotated_by_joint[0] + _link3_rotated_by_joint[0] )
                                )
        
        # Draw The EE dot automatically
        _EEDrawDot = [ math.cos(self.dobotJoints[0]) * ( _link2_rotated_by_joint[0] + _link3_rotated_by_joint[0] + self.EE_XY_len ), 
                      -math.sin(self.dobotJoints[0]) * ( _link2_rotated_by_joint[0] + _link3_rotated_by_joint[0] + self.EE_XY_len )]
        self.EEDrawDot.setPos(_EEDrawDot[0], _EEDrawDot[1])
        
        _CDot = [  self.DrawXCoordinateSliderCircumference.value() * self.sceneScaling + math.cos(self.dobotJoints[0])*self.EE_XY_len,
                 -(self.DrawYCoordinateSliderCircumference.value() * self.sceneScaling + math.sin(self.dobotJoints[0])*self.EE_XY_len)]
        self.CDot.setPos(_CDot[0], _CDot[1])
        
        _EndDot = [  self.DrawXCoordinateSliderEnd.value() * self.sceneScaling + math.cos(self.dobotJoints[0]) * self.EE_XY_len,
                   -(self.DrawYCoordinateSliderEnd.value() * self.sceneScaling + math.sin(self.dobotJoints[0]) * self.EE_XY_len)]
        self.EndDot.setPos(_EndDot[0], _EndDot[1])

        if self.Root.currentIndex() != 3 or self.ProgrammingRoot.currentIndex() != 2: # This is the drawing tab
            self.CDot.hide()
            self.EndDot.hide()
        else:
            self.CDot.show()
            self.EndDot.show()

    def tcp_position_callback(self, msg):
        self.dobotPose = [msg.data[0]*1000, msg.data[1]*1000, msg.data[2]*1000, msg.data[3]]
        self.XPoseLCD.setText(str(round(self.dobotPose[0], 3)))
        self.YPoseLCD.setText(str(round(self.dobotPose[1], 3)))
        self.ZPoseLCD.setText(str(round(self.dobotPose[2], 3)))
        self.RPoseLCD.setText(str(round(self.dobotPose[3], 3)))


    def joints_positions_callback(self, msg):
        self.dobotJoints = msg.position # Stores joint position in radians
        self.JT1LCD.setText(str(round((math.degrees(self.dobotJoints[0])), 3)))
        self.JT2LCD.setText(str(round((math.degrees(self.dobotJoints[1])), 3)))
        self.JT3LCD.setText(str(round((math.degrees(self.dobotJoints[2])), 3)))
        self.JT4LCD.setText(str(round((math.degrees(self.dobotJoints[3])), 3)))

    def rail_pose_callback(self, msg):
        self.rail_pose = msg.data
        self.CurrentPositionRail.setText(str(int(self.rail_pose)))

    def gripper_control_tab(self, b):
        if b.text() == "Close" and b.isChecked() == True:
            self.gripperClose = True
            self.EEGripperOpen.setChecked(False)
                    
        if b.text() == "Open" and b.isChecked() == True:
            self.gripperClose = False
            self.EEGripperClose.setChecked(False)
        
        if self.gripperType != "gripper":
            self.EESuctionCupOn.setChecked(False)
            self.EESuctionCupOff.setChecked(False)
        self.gripperType = "gripper"
    
    def suctionCup_control_tab(self, b):
        if b.text() == "On" and b.isChecked() == True:
            self.suctionCupOn = True
            self.EESuctionCupOff.setChecked(False)
                    
        if b.text() == "Off" and b.isChecked() == True:
            self.suctionCupOn = False
            self.EESuctionCupOn.setChecked(False)
        
        if self.gripperType != "suctionCup":
            self.EEGripperClose.setChecked(False)
            self.EEGripperOpen.setChecked(False)
        
        self.gripperType = "suctionCup"

    def framestate_control_tab(self, b):
        if b.text() == "Base" and b.isChecked() == True:
            self.frame = "base"
                    
        if b.text() == "Joint" and b.isChecked() == True:
            self.frame = "joint"

    def framestate_programming_tab_valueType(self, b):
        if b.text() == "Base" and b.isChecked() == True:
            self.valueType = "base"
                    
        if b.text() == "Joint" and b.isChecked() == True:
            self.valueType = "joint"

    def framestate_programming_tab_movement(self, b):
        if b.text() == "MoveL" and b.isChecked() == True:
            self.movementType = "movel"
                    
        if b.text() == "MoveJ" and b.isChecked() == True:
            self.movementType = "movej"
            

    def JT_IDLE(self):
        if self.frame == "base":
            bot.set_jog_command(0, 0)
        elif self.frame == "joint":
            bot.set_jog_command(1, 0)

        
    def JT1_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 1)
            elif self.frame == "joint":
                bot.set_jog_command(1, 1)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 2)
            elif self.frame == "joint":
                bot.set_jog_command(1, 2)

    def JT2_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 3)
            elif self.frame == "joint":
                bot.set_jog_command(1, 3)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 4)
            elif self.frame == "joint":
                bot.set_jog_command(1, 4)

    def JT3_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 5)
            elif self.frame == "joint":
                bot.set_jog_command(1, 5)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 6)
            elif self.frame == "joint":
                bot.set_jog_command(1, 6)

    def JT4_move(self, sign):
        if sign.text() == "+":
            if self.frame == "base":
                bot.set_jog_command(0, 7)
            elif self.frame == "joint":
                bot.set_jog_command(1, 7)
        elif sign.text() == "-":
            if self.frame == "base":
                bot.set_jog_command(0, 8)
            elif self.frame == "joint":
                bot.set_jog_command(1, 8)

    def button_clicked_HomingButton(self):   
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        # msg.setText("Information")
        msg.setInformativeText("The homing procedure is about to start. Wait until the arm stops moving and the led stops flashing blue and turns green.")
        msg.setWindowTitle("Homing procedure")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

        command = 'ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def Arc_teach_command(self):
        # -------------------------- #
        # Teach the pose in Arc      #
        # -------------------------- #
        circumference_point = [self.DrawXCoordinateAxisCircumference.text(), 
                               self.DrawYCoordinateAxisCircumference.text(), 
                               self.DrawZCoordinateAxisCircumference.text(), 
                               0.0]
        ending_point = [self.DrawXCoordinateAxisEnd.text(), 
                               self.DrawYCoordinateAxisEnd.text(), 
                               self.DrawZCoordinateAxisEnd.text(), 
                               0.0]
        command_file = self.programmingLineEditCommandPath.text()
        #command_path = '/home/vboxuser/ws_magician/src/dobot_menu/resource/commands/commands.bash'
        command_motion = "{circumference_point: " + str(circumference_point) + ", ending_point: " + str(ending_point) + ", velocity_ratio: 0.5, acceleration_ratio: 0.3}"
        command_cmd = "ros2 action send_goal /Arc_action dobot_msgs/action/ArcMotion " + "\\\"" + command_motion +  "\\\"" + " --feedback"
        command = 'echo ' + command_cmd + ' >> ' + self.import_path + '/' + command_file
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,bufsize=1).communicate()
        
        self.execute_load_command(self.programmingLineEditCode) # Load the imported code to file

    def EE_teach_command(self):
        # -------------------------- #
        # Teach end-effector command #
        # -------------------------- #

        if self.gripperType == "gripper" and self.gripperClose == True:
            command_cmd = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "close", keep_compressor_running: false}"'
            self.send_gripper_state("opened")
        elif self.gripperType == "gripper" and self.gripperClose == False:
            command_cmd = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "open", keep_compressor_running: false}"'
            self.send_gripper_state("opened")
        elif self.gripperType == "suctionCup" and self.suctionCupOn == True:
            command_cmd = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"'
            self.send_gripper_state("opened")
        elif self.gripperType == "suctionCup" and self.suctionCupOn == False:
            command_cmd = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: false}"'
            self.send_gripper_state("opened")
        
        command_file = self.programmingLineEditCommandPath.text()
        command = 'echo ' + command_cmd + ' >> ' + self.import_path + '/' + command_file
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,bufsize=1).communicate()
        
        self.execute_load_command(self.programmingLineEditCode) # Load the imported code to file
    
    def PTP_teach_command(self):
        # --------------------- #
        # Teach the pose in PTP #
        # --------------------- #
        motion_type = 1
        dobotMotion = self.dobotPose
        if self.movementType == "movel":
            motion_type = motion_type + 1
        if self.valueType == "joint":
            motion_type = motion_type + 3
            dobotMotion = [math.degrees(self.dobotJoints[0]), math.degrees(self.dobotJoints[1]), math.degrees(self.dobotJoints[2]), math.degrees(self.dobotJoints[3])]
        command_file = self.programmingLineEditCommandPath.text()
        #command_path = '/home/vboxuser/ws_magician/src/dobot_menu/resource/commands/commands.bash'
        command_motion = "{motion_type: " + str(motion_type) + ", target_pose: " + str(dobotMotion) + ", velocity_ratio: 0.5, acceleration_ratio: 0.3}"
        command_cmd = "ros2 action send_goal /PTP_action dobot_msgs/action/PointToPoint " + "\\\"" + command_motion +  "\\\"" + " --feedback"
        command = 'echo ' + command_cmd + ' >> ' + self.import_path + '/' + command_file
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,bufsize=1).communicate()
        
        self.execute_load_command(self.programmingLineEditCode) # Load the imported code to file
    
    def execute_command(self):
        # ------------------- #
        # Execute the code    #
        # ------------------- #
        command_file = self.programmingLineEditCommandPath.text()
        #command = 'bash ~/ws_magician/src/dobot_menu/resource/commands/commands.bash'
        command = 'bash ' + self.import_path + '/' + command_file
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def export_command(self):
        # ------------------- #
        # Export code to PC   #
        # ------------------- #
        export_path = self.programmingLineEditExportPath.text()
        command_file = self.programmingLineEditCommandPath.text()
        command = 'cp ' + self.import_path + '/' + command_file + ' ' + export_path
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def import_command(self):
        # ------------------- #
        # Import code from PC #
        # ------------------- #
        export_path = self.programmingLineEditExportPath.text()
        command_file = self.programmingLineEditCommandPath.text()
        command = 'cp ' + export_path + '/' + command_file + ' -r ' + self.import_path
        #command = 'cp /mnt/c/users/mathias/WSL/commands.bash -r ~/ws_magician/src/dobot_menu/resource/commands'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        
        self.execute_load_command(self.programmingLineEditCode) # Load the imported code to file
        
    def execute_load_command(self, widget):
        # ------------------- #
        # Load code from file #
        # ------------------- #
        command_file = self.programmingLineEditCommandPath.text()
        self.programmingLabelCode.setText(command_file)             # 

        try:
            with open(self.import_path + '/' + command_file, 'r') as file:
                code = file.read()
                widget.setText(code) # self.programmingLineEditCode
        except Exception as e:
            widget.setText(f"Error saving file: {e}") # self.programmingLineEditCode

    def execute_save_command(self, widget):
        # ------------------- #
        # Save code to file   #
        # ------------------- #
        command_file = self.programmingLineEditCommandPath.text()
        code = widget.toPlainText()
        try:
            with open(self.import_path + '/' + command_file, 'w') as file:
                file.write(code)
        except Exception as e:
            self.programmingLineEditCode.setText(f"Error saving file: {e}")
            

    def button_clicked_EStopButton(self):
        bot.stop_queue(force=True) 

        command = 'ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Warning")
        msg.setInformativeText("The stop button has been pressed. Make sure nothing is in the robot's workspace before resuming operation.")
        msg.setWindowTitle("Motion stop")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
        bot.clear_alarms_state()


    def open_gripper(self):
        command = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "open", keep_compressor_running: false}"'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        self.send_gripper_state("opened")
 
    def close_gripper(self):
        command = 'ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: "close", keep_compressor_running: false}"'
        subprocess.Popen(
                    command, universal_newlines=True, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

        self.send_gripper_state("closed")


    def turn_on_suction_cup(self):
        command = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"'
        subprocess.Popen(
            command, universal_newlines=True, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def turn_off_suction_cup(self):
        command = 'ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: false}"'
        subprocess.Popen(
            command, universal_newlines=True, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def valuechange_joints(self, field):
        if field.objectName() == "JT1Vel":
            self.JT1Vel.setText(str(self.Joint1VelSlider.value()))
        elif field.objectName() == "JT2Vel":
            self.JT2Vel.setText(str(self.Joint2VelSlider.value()))
        elif field.objectName() == "JT3Vel":
            self.JT3Vel.setText(str(self.Joint3VelSlider.value()))
        elif field.objectName() == "JT4Vel":
            self.JT4Vel.setText(str(self.Joint4VelSlider.value()))

        elif field.objectName() == "JT1Acc":
            self.JT1Acc.setText(str(self.Joint1AccSlider.value()))
        elif field.objectName() == "JT2Acc":
            self.JT2Acc.setText(str(self.Joint2AccSlider.value()))
        elif field.objectName() == "JT3Acc":
            self.JT3Acc.setText(str(self.Joint3AccSlider.value()))
        elif field.objectName() == "JT4Acc":
            self.JT4Acc.setText(str(self.Joint4AccSlider.value()))


    def valuechange_cartesian(self, field):
        if field.objectName() == "XVel":
            self.XVel.setText(str(self.XVelSlider.value()))
        elif field.objectName() == "YVel":
            self.YVel.setText(str(self.YVelSlider.value()))
        elif field.objectName() == "ZVel":
            self.ZVel.setText(str(self.ZVelSlider.value()))
        elif field.objectName() == "RVel":
            self.RVel.setText(str(self.RVelSlider.value()))
        if field.objectName() == "XAcc":
            self.XAcc.setText(str(self.XAccSlider.value()))
        elif field.objectName() == "YAcc":
            self.YAcc.setText(str(self.YAccSlider.value()))
        elif field.objectName() == "ZAcc":
            self.ZAcc.setText(str(self.ZAccSlider.value()))
        elif field.objectName() == "RAcc":
            self.RAcc.setText(str(self.RAccSlider.value()))

    def change_vel_joints(self, slider):
        bot.set_jog_joint_params([int(self.Joint1VelSlider.value()), 
                                  int(self.Joint2VelSlider.value()), 
                                  int(self.Joint3VelSlider.value()), 
                                  int(self.Joint4VelSlider.value())], 

                                 [int(self.Joint1AccSlider.value()), 
                                  int(self.Joint2AccSlider.value()), 
                                  int(self.Joint3AccSlider.value()), 
                                  int(self.Joint4AccSlider.value())])


    def change_vel_cartesian(self, slider):
        bot.set_jog_coordinate_params([int(self.XVelSlider.value()), 
                                       int(self.YVelSlider.value()), 
                                       int(self.ZVelSlider.value()), 
                                       int(self.RVelSlider.value())], 

                                      [int(self.XAccSlider.value()), 
                                       int(self.YAccSlider.value()), 
                                       int(self.ZAccSlider.value()), 
                                       int(self.RAccSlider.value())])


    def sliding_rail_disconnected(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Sliding rail is not connected")
        msg.setInformativeText("If you have actually placed the manipulator on the sliding rail, you must additionally set MAGICIAN_RAIL_IN_USE environment variable to 'true'.")
        msg.setWindowTitle("WARNING")
        msg.setStandardButtons(QMessageBox.Close)
        msg.exec_()

    def sliding_rail_SV_slider(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return

        bot.set_point_to_point_sliding_rail_command(4,math.degrees(self.dobotJoints[0]), math.degrees(self.dobotJoints[1]), 
                                                    math.degrees(self.dobotJoints[2]), math.degrees(self.dobotJoints[3]), 
                                                    int(self.RailSVSlider.value()))

    def sliding_rail_current_pose(self):
        self.RailSVDisplay.setText(str(self.RailSVSlider.value()))

    def sliding_rail_params_slider(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return
        bot.set_point_to_point_sliding_rail_params(int(self.RailVelocitySlider.value()) * 2, int(self.RailAccelerationSlider.value()) * 2) #BUG

    def sliding_rail_vel_display(self):
        self.RailVelLCD.setText(str(self.RailVelocitySlider.value()))


    def sliding_rail_acc_display(self):
        self.RailAccLCD.setText(str(self.RailAccelerationSlider.value()))

    def button_clicked_StopRail(self):
        if not self.RAIL_IN_USE:
            self.sliding_rail_disconnected()
            return
        bot.stop_queue(force=True) 
        bot.clear_queue()
        bot.start_queue()

        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)

        msg.setText("Warning")
        msg.setInformativeText("The stop button has been pressed. Make sure nothing is in the robot's workspace before resuming operation.")
        msg.setWindowTitle("Motion stop")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    # --------------------------------------------------------
    def send_gripper_state(self, state):
        msg = GripperStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.status = state
        self.gripper_state_publ.publish(msg)


          
    def start(self):
        pass

    def shutdown_plugin(self):
        pass
