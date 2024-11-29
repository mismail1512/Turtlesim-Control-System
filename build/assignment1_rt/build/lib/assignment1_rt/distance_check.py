import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # Correct message type for turtlesim
from geometry_msgs.msg import Twist
from math import sqrt

class DistanceCheck(Node):
    def __init__(self):
        super().__init__('distance_check')

        # Publishers for turtle1 and turtle2 velocities
        self.publisher_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Subscribers for turtle1 and turtle2 poses
        self.subscription_turtle1 = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback_turtle1, 10)
        self.subscription_turtle2 = self.create_subscription(Pose, '/turtle2/pose', self.pose_callback_turtle2, 10)

        # Initialize turtle positions
        self.pose_turtle1 = None  # Pose of turtle1
        self.pose_turtle2 = None  # Pose of turtle2

        # Set distance threshold and boundary limits
        self.distance_threshold = 1.0  # Minimum distance to stop if too close
        self.boundary_limit = 10.0  # Boundary limit for the turtles

    def pose_callback_turtle1(self, msg):
        """Callback for turtle1's pose."""
        self.pose_turtle1 = msg
        self.check_distance_and_boundaries()

    def pose_callback_turtle2(self, msg):
        """Callback for turtle2's pose."""
        self.pose_turtle2 = msg
        self.check_distance_and_boundaries()

    def check_distance_and_boundaries(self):
        """Check distance between turtles and boundaries, stop them if needed."""
        # Ensure we have valid poses for both turtles
        if self.pose_turtle1 is None or self.pose_turtle2 is None:
            return

        # Calculate the distance between turtle1 and turtle2
        distance = sqrt(
            (self.pose_turtle1.x - self.pose_turtle2.x) ** 2 +
            (self.pose_turtle1.y - self.pose_turtle2.y) ** 2
        )

        # Check proximity
        if distance < self.distance_threshold:
            self.stop_robots()
            self.get_logger().info(f"Turtles are too close! Distance: {distance:.2f}")

        # Check if turtles are near the boundaries
        if not (1.0 < self.pose_turtle1.x < self.boundary_limit and 1.0 < self.pose_turtle1.y < self.boundary_limit):
            self.stop_robots()
            self.get_logger().info("Turtle1 is out of bounds!")

        if not (1.0 < self.pose_turtle2.x < self.boundary_limit and 1.0 < self.pose_turtle2.y < self.boundary_limit):
            self.stop_robots()
            self.get_logger().info("Turtle2 is out of bounds!")

    def stop_robots(self):
        """Stop both turtles by sending zero velocities."""
        stop_msg = Twist()  # Create a zero velocity message
        self.publisher_turtle1.publish(stop_msg)
        self.publisher_turtle2.publish(stop_msg)
        self.get_logger().info("Turtles stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Distance Check node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
