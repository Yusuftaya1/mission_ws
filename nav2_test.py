#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def creater_pese_stamped(navigator:BasicNavigator ,pose_x,pose_y,orientation_z):
    q_x, q_y, q_z, q_w= tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pose_x
    pose.pose.position.y = pose_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.x = q_y
    pose.pose.orientation.x = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    ##--init
    rclpy.init()
    nav = BasicNavigator()

    # --- set init pose
    inital_pose= creater_pese_stamped(nav,0.0,0.0,0.0)
    nav.setInitialPose(inital_pose)


    #-Wait for Nav2
    nav.waitUntilNav2Active()

    #-Send Goal
    goal1 = creater_pese_stamped(nav, 2.5, 1.5, 1.57)
    goal2 = creater_pese_stamped(nav, 2.5, -1.5, 1.57)
    goal3 = creater_pese_stamped(nav, 2.0, 2.5, 1.57)

    waypoints=[goal1,goal2,goal3]
    nav.followWaypoints(waypoints)
    
    while not nav.isTaskComplete():
        feedback= nav.getFeedback()
        #print(feedback)

    print(nav.getResult())

    #shutdown
    rclpy.shutdown()


if __name__ =='__main__':
    main()