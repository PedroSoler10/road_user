import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReactorNode(Node):
    def __init__(self):
        super().__init__('reactor_node')
        self.publisher_ = self.create_publisher(String, 'reaction', 10)
        self.subscription = self.create_subscription(String,'scenario', self.scenario_callback, 10)
        self.subscription  # prevent unused variable warning

    def scenario_callback(self, msg):
        reaction = String()
        if msg.data == "used":
            reaction.data = "wait"
        elif msg.data == "free":
            reaction.data = "continue"
        else:
            reaction.data = "unknown"
        
        self.publisher_.publish(reaction)
        self.get_logger().info(f"The selected reaction is: {reaction.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReactorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
