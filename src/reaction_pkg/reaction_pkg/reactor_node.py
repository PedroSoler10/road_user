#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped, StampedPose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class ReactorNode(Node):
    def __init__(self):
        super().__init__('reactor_node')

        self.publisher_emergency = self.create_publisher(AckermannDriveStamped, '/emergency', 30)
        self.publisher_goal_pose = self.create_publisher(StampedPose, '/goal_pose', 30)

        self.emergency = AckermannDriveStamped()
        self.slow = False
        self.trans = TransformStamped()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription_scenario = self.create_subscription(String,'/scenario', self.scenario_callback, 10)
        self.subscription_goal_pose = self.create_subscription(StampedPose,'/goal_pose', self.goal_pose_callback, 10)
        self.subscription_drive = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)

    def init_zones(self):
        # 1st init
        self.init_1 = StampedPose()
        self.init_1.pose.position.x = 9.5
        self.init_1.pose.position.y = 3.6

        # 1st stop
        self.stop_1 = StampedPose()
        self.stop_1.pose.position.x = 13.2
        self.stop_1.pose.position.y = 3.6
        self.stop_2.pose.orientation.w = 1

        # 2nd init
        self.init_2 = StampedPose()
        self.init_2.pose.position.x = 14.9
        self.init_2.pose.position.y = 3.6

        # 2nd stop
        self.stop_2 = StampedPose()
        self.stop_2.pose.position.x = 18.3
        self.stop_2.pose.position.y = 6.5
        self.stop_2.pose.orientation.z = 0.7
        self.stop_2.pose.orientation.w = -0.7

        self.zona = 1

    def goal_pose_callback(self, msg):
        if msg.pose.point.x < self.init_1.pose.position.x or msg.pose.point.x > self.stop_2.pose.position.x:
            msg = self.ext_pose

    def drive_callback(self, msg):
        if self.slow is True and msg.drive.speed > 0:
            msg = self.emergency
            self.emergency.drive.speed = 0.4
            self.publisher_emergency.publish(self.emergency)

    def get_map_to_base_link_transform(self):
        try:
            # Espera por la transformación a estar disponible
            self.trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('No se pudo obtener la transformada de map a base_link: %s' % str(e))
        
    def scenario_callback(self, msg):
        if msg.data is not "free":
            self.get_map_to_base_link_transform()
            # Conflictive zone
            if self.trans.transform.translation.x > self.init_1.x:
                # Before last stop
                if self.trans.transform.translation.y > self.stop_2.y:
                    self.zona = 1
                elif self.trans.transform.translation.x > self.init_2.x:
                    self.zona = 2
                elif self.trans.transform.translation.x > self.stop_1.x:
                    self.zona = 3
                else:
                    self.zona = 4
            else:
                self.zona = 5
            
            if self.zona is 2:
                self.goal = self.stop_2
                self.slow = True
            elif self.zona is 4 :
                if msg.data is "used":
                    self.goal = self.stop_1
                    self.slow = True
                else:
                    self.slow = True
            else:
                self.slow = False
        else:
            self.slow = False
            self.goal = self.init_goal
        
        self.publisher_goal_pose.publish(self.goal)

def main(args=None):
    rclpy.init(args=args)
    node = ReactorNode()
    rclpy.spin(node)
    node.destroy_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
