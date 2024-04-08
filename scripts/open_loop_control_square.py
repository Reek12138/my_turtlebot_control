#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move", anonymous=True)
        rospy.loginfo("Press Ctrl + C to terminate")
        # Adjust the topic according to your Turtlebot version/configuration
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.sleep(4)

        self.run()

    def forward(self, distance):
        vel = Twist()
        vel.linear.x = 0.2  # Adjust linear speed if needed
        travel_time = distance / vel.linear.x

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < travel_time:
            self.vel_pub.publish(vel)
            self.rate.sleep()

        # Stop the robot after moving forward
        vel.linear.x = 0
        self.vel_pub.publish(vel)

    def turn(self, angle):
        vel = Twist()
        vel.angular.z = 0.5  # Adjust angular speed if needed
        turn_time = angle / vel.angular.z

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < turn_time:
            self.vel_pub.publish(vel)
            self.rate.sleep()

        # Stop the robot after turning
        vel.angular.z = 0
        self.vel_pub.publish(vel)

    def run(self):
        # Move in a square
        for _ in range(4):
            self.forward(2)  # Adjust the distance to match your requirement
            rospy.sleep(1)
            self.turn(pi/2)  # pi/2 radians = 90 degrees
            rospy.sleep(1)


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")

