#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ApriltagModule import Detector
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

class Missions(Node):
    def __init__(self):
        super().__init__('missions_node')
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.nav = BasicNavigator()

    def create_pose_stamped(self, pose_x, pose_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def publish_goal_pose(self, pose):
        self.publisher.publish(pose)
        self.get_logger().info("Goal pose published")

    def mission0(self):
        goal_pose = self.create_pose_stamped(2.0, 0.0, 1.57)
        self.publish_goal_pose(goal_pose)
        self.get_logger().info("MISSION 0: Goal pose published")

    def mission1(self):
        print("MISSION 1 TAG DETECT")

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.tag_detector = Detector()
        self.missions = Missions()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray)

        for tag in detections:
            tag_id = tag.tag_id
            print(tag_id)

            if tag_id == 0:
                self.missions.mission0()

            elif tag_id == 1:
                self.missions.mission1()

def main(args=None):
    rclpy.init(args=args)

    april_tag_detector = AprilTagDetector()
    rclpy.spin(april_tag_detector)

    april_tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
