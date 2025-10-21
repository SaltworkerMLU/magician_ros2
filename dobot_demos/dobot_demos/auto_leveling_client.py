from dobot_msgs.srv import ExecuteAutoLevelingProcedure
import rclpy
from rclpy.node import Node


class AutoLevelingClient(Node):

    def __init__(self):
        super().__init__('dobot_homing_cli')
        self.auto_leveling_cli = self.create_client(ExecuteAutoLevelingProcedure, 'dobot_auto_leveling_service')
        while not self.auto_leveling_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Auto leveling service not available, waiting again...')
        self.auto_leveling_req = ExecuteAutoLevelingProcedure.Request()
    
    def send_request_auto_leveling(self):
        self.auto_leveling_future = self.auto_leveling_cli.call_async(self.auto_leveling_req)
        self.get_logger().info("Auto Leveling has started.")
        rclpy.spin_until_future_complete(self, self.auto_leveling_future)
        return self.auto_leveling_future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = AutoLevelingClient()
    response = minimal_client.send_request_auto_leveling()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


