#! /usr/bin/env python3

import math
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_rb1_ros.srv import Rotate, RotateRequest, RotateResponse
from tf.transformations import euler_from_quaternion


class RotateService:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.srv = rospy.Service('/rotate_robot', Rotate, self.rotate_robot)
        self.current_orientation = None

        rospy.loginfo("Rotate Service Ready. Waiting for rotation requests...")
        rospy.spin()

    def odom_callback(self, msg):
        """
        Callback to update the robot's current odometry.
        Extracts the robot's current orientation in quaternion form and converts it to Euler angles.
        """
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def rotate_robot(self, request):
        rospy.loginfo(f"Rotate Service Requested: {request.degrees} [deg].")
        rospy.loginfo(f"Current orientation: {self.current_orientation * 180.0 / math.pi:.2f} [deg].")
        target_angle = self.current_orientation + (request.degrees * 3.141 / 180.0)

        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi

        twist = Twist()
        if request.degrees > 0:
            twist.angular.z = 0.4
        else:
            twist.angular.z = -0.4

        rate = rospy.Rate(10)
        start_time = time.time()
        while not rospy.is_shutdown():
            if abs(self.current_orientation - target_angle) < 0.05:
                break
            if time.time() - start_time > 10: # Failure case
                twist.angular.z = 0
                self.pub.publish(twist)
                rospy.logwarn(f"Rotation incomplete. Final orientation: {self.current_orientation * 180.0 / math.pi:.2f} [deg]")
                return RotateResponse(result=f"Failed: Rotation incomplete. Final orientation: {self.current_orientation * 180.0 / math.pi:.2f} [deg]")
            
            self.pub.publish(twist)
            rate.sleep()

        twist.angular.z = 0
        self.pub.publish(twist)
        rospy.loginfo(f"Rotate Service Completed. Target orientation reached: {target_angle * 180.0 / math.pi:.2f} [deg]")
        return RotateResponse(result=f"Rotate Service Completed. Target orientation reached: {target_angle * 180.0 / math.pi:.2f} [deg]")


if __name__ == '__main__':
    rospy.init_node('rotate_robot_service')
    rotate_service = RotateService()
    rospy.spin()
