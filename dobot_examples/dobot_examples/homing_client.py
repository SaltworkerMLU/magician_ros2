from dobot_msgs.srv import ExecuteHomingProcedure, ExecuteAutoLevelingProcedure
import rclpy
from rclpy.node import Node


class HomingClient(Node):

    def __init__(self):
        super().__init__('dobot_homing_cli')
        self.cli = self.create_client(ExecuteHomingProcedure, 'dobot_homing_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ExecuteHomingProcedure.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.get_logger().info("The homing procedure has started. Please wait until the arm stops moving and the led stops flashing blue.")
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = HomingClient()
    response = minimal_client.send_request()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


