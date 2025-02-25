import rclpy
from rclpy.node import Node
from my_own_srv.srv import Moveturtlebot3
from geometry_msgs.msg import Twist
import time
import math

class ServerClass(Node):
    def __init__(self):
        super().__init__('turtlebot3_server')
        self.srv = self.create_service(Moveturtlebot3, 'move_to_position', self.go_to_position_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Service Server Ready!")

    def go_to_position_callback(self, request, response):
        start_time = time.time()

        # Calculate the angle to rotate towards the target
        theta = math.atan2(request.y, request.x)  # Angle in radians
        twist_msg = Twist()
        # Rotate to face the correct direction
        twist_msg.angular.z = 0.5  # Positive means counterclockwise rotation
        rotate_time = abs(theta) / 0.5  # Time needed to rotate at 0.5 rad/s
        self.cmd_vel_pub.publish(twist_msg)
        time.sleep(rotate_time)  # Wait for rotation to complete

        # Stop rotating
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        # Move forward in that direction
        distance = math.sqrt(request.x**2 + request.y**2)  # Distance to target
        twist_msg.linear.x = 0.2  # Move forward
        move_time = distance / 0.2  # Time needed to move at 0.2 m/s
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Moving to ({request.x}, {request.y}), Estimated Time: {move_time}s")

        time.sleep(move_time)  # Simulate travel time

        # Stop the robot
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        response.taken_time = time.time() - start_time
        self.get_logger().info(f"Reached Destination! Time Taken: {response.taken_time}s")
        return response

def main():
    rclpy.init()
    node = ServerClass()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
