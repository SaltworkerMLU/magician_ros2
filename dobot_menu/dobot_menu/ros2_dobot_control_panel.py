from rqt_gui_py.plugin import Plugin
from .dobot_control_panel_widget import DobotControlPanel


class Ros2DobotControlPanel(Plugin):

    def __init__(self, context):
        super(Ros2DobotControlPanel, self).__init__(context)    # Call inherited __init__ methods
        self._node = context.node                               # Store rclpy.node.Node object
        self._logger = self._node.get_logger().get_child(
            'dobot_menu.ros2_dobot_control_panel.Ros2DobotControlPanel'
        )
        self.setObjectName('Ros2DobotControlPanel')

        self._widget = DobotControlPanel(context.node, self)

        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
