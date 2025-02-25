import rclpy
from rclpy.node import Node
from my_own_srv.srv import Moveturtlebot3

class ClientClass(Node):
    def __init__(self):
        super().__init__('turtlebot3_client')
        self.client = self.create_client(Moveturtlebot3, 'move_to_position')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for service...")
        self.get_logger().info("Service available!")

    def send_goal(self, x, y):
        request = Moveturtlebot3.Request()
        request.x = x
        request.y = y
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ClientClass()
    x = float(input("Enter x coordinate: "))
    y = float(input("Enter y coordinate: "))
    response = node.send_goal(x, y)
    print(f"Time taken to reach: {response.taken_time} seconds")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
