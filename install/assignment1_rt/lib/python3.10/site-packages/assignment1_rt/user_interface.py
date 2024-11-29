import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time  # For proper delay

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.velocity = Twist()

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher.publish(self.velocity)  # Stop the robot
        self.get_logger().info("Robot stopped.")

    def set_velocity(self):
        """Get user input for velocity and publish to the selected robot."""
        try:
            robot = input("Which turtle do you want to control? (turtle1 (t1)/turtle2 (t2)): ").strip()
            if robot not in ["t1", "t2"]:
                self.get_logger().error("Invalid turtle name. Use 't1' or 't2'.")
                return

            linear_speed = float(input("Enter linear speed: "))
            angular_speed = float(input("Enter angular speed: "))

            # Update the publisher based on the selected turtle
            topic = '/turtle2/cmd_vel' if robot == "t2" else '/turtle1/cmd_vel'
            self.publisher = self.create_publisher(Twist, topic, 10)

            # Publish the velocity command
            self.velocity.linear.x = linear_speed
            self.velocity.angular.z = angular_speed
            self.publisher.publish(self.velocity)

            self.get_logger().info(f"Controlling {robot} with linear speed {linear_speed} and angular speed {angular_speed}")

            # Wait for 1 second before stopping the robot
            time.sleep(1)  # Use time.sleep for a blocking delay
            self.stop_robot()
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values for speed.")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()

    try:
        while rclpy.ok():
            node.set_velocity()  # Continuously prompt for user input
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down User Interface node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
