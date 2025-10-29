from dobot_msgs.srv import ExecuteHomingProcedure
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rclpy.parameter import Parameter
import math


class CPService(Node):

    def __init__(self):
        super().__init__('dobot_CP_srv')
        self.srv = self.create_service(ExecuteHomingProcedure, 'dobot_CP_service', self.CP_callback)

    def CP_callback(self, request, response):
        

        [start_x, start_y, start_z, start_r] = bot.get_pose()[0:4]

        self.get_logger().info(str([start_x, start_y, start_z, start_r]))

        bot.set_continous_trajectory_real_time_params(20, 100, 10)

        # Draw about half an arch as a single path
        bot.stop_queue()
        bot.clear_queue()
        steps = 24
        scale = 25
        for i in range(steps + 2):
            x = math.cos((2*math.pi / steps) * i)
            y = math.sin((2*math.pi / steps) * i)
            #bot.set_point_to_point_command(1, start_x + x * scale, start_y + y * scale, start_z, 50, queue=True)
            bot.set_continous_trajectory_command(1, start_x + x * scale, start_y + y * scale, start_z, 50, queue=True)

            # Absolute movement
        
        bot.start_queue()
        #time.sleep(2)

        response.success = True
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = CPService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()