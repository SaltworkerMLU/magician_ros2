from dobot_msgs.srv import ExecuteAutoLevelingProcedure
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time


class AutoLeveling(Node):

    def __init__(self):
        super().__init__('dobot_auto_leveling_srv')
        self.srv = self.create_service(ExecuteAutoLevelingProcedure, 
                                       'dobot_auto_leveling_service', 
                                       self.auto_leveling_callback)

    # This function is executed when the service 'dobot_auto_leveling_service' is prompted
    def auto_leveling_callback(self, request, response):
        bot.set_auto_leveling(1, 25)
        time.sleep(0.1)
        re = bot.get_auto_leveling()
        self.get_logger().info(re)

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = AutoLeveling()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()