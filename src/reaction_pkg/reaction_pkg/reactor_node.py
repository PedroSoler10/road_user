import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class ReactorNode(Node):
    def __init__(self):
        super().__init__('reactor_node')
        self.publisher_reaction = self.create_publisher(String, 'reaction', 10)
        self.publisher_emergency = self.create_publisher(AckermannDriveStamped, 'emergency', 30)
        self.subscription = self.create_subscription(String,'scenario', self.scenario_callback, 10)
        self.reaction_state = String()
        self.emergency_cmd = AckermannDriveStamped()

    def scenario_callback(self, msg):
        if msg.data == "used":
            self.reaction_state.data = "wait"

        elif msg.data == "free":
            self.reaction_state.data = "continue"
        else:
            self.reaction_state.data = "unknown"
        
        self.publisher_reaction.publish(self.reaction_state)
        self.get_logger().info(f"The reaction state is: {self.reaction_state.data}")
        self.emergency_check()
    
    def emergency_check(self):
        if self.reaction_state.data == "wait":
            self.emergency_cmd.drive.speed = 0.0
            self.emergency_cmd.drive.steering_angle = 0.0
            self.publisher_emergency.publish(self.emergency_cmd)
            self.get_logger().info(f"The emergency command is:\n Steering angle: {self.emergency_cmd.drive.steering_angle}\nSpeed: {self.emergency_cmd.drive.speed}")

def main(args=None):
    rclpy.init(args=args)
    node = ReactorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
