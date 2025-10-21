#!/usr/bin/env python3

import sys
from rqt_gui.main import Main

"""
The "main()" script is in this case calling the 
"""
def main():
    main = Main() # call rqt_gui.main object
    sys.exit( # Exit when...
             main.main( # main application [...] has run its course
                       sys.argv, standalone='dobot_menu.ros2_dobot_control_panel.Ros2DobotControlPanel'
                      )
            )

if __name__ == '__main__':
    main()