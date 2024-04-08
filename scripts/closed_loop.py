#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from enum import Enum
import threading

class mode(Enum):
    init = 0
    forward = 1
    turn = 2
    reach_goal = 3


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.mode = mode.init

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        # add your code here
        waypoints = [  [4,0], [4,4],[0,4], [0,0]]
        angle_tolerance = 0.005
        distance_tolerance = 0.05

        angle_controller = Controller(P=0.75, D=1.0, output_limits=(-2,2))
        

        # for waypoint in waypoints:
        #     self.mode = mode.init
        #     target_angle = atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x)
        #     angle_controller.setPoint(target_angle)
        waypoint = waypoints[0]
        while not rospy.is_shutdown():
                
                if self.mode == mode.init:
                    print("INIT..................................................")
                    current_angle = self.pose.theta
                    target_angle = atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x)
                    angle_error = atan2(sin(target_angle - current_angle), cos(target_angle - current_angle))
                    
                    if abs(angle_error) < angle_tolerance:
                        self.mode = mode.forward  
                    else:
                        self.mode = mode.turn

                elif self.mode == mode.forward:
                    print("FORWARD...............................................")
                    print("waypoint:",waypoint)
                    distance_controller = Controller(P=1.0, D=0.3,output_limits=(-0.5, 0.5))
                    distance = sqrt((waypoint[0] - self.pose.x) ** 2 + (waypoint[1] - self.pose.y) ** 2)
                    print("distance:",distance)
                    if distance < distance_tolerance:
                        if waypoints.index(waypoint) == len(waypoints) - 1:  
                            self.mode = mode.reach_goal
                        else:
                            self.mode = mode.turn  
                            next_point = waypoints[(waypoints.index(waypoint) + 1) % len(waypoints)] 
                            waypoint = next_point
                            self.control(self.vel_pub, 0, 0) 
                        # self.mode = mode.turn  
                        # next_point = waypoints[(waypoints.index(waypoint) + 1) % len(waypoints)] 
                        # waypoint = next_point
                        # self.control(self.vel_pub, 0, 0)  
                    else:
                        linear_speed = distance_controller.update(-distance)

                        current_angle = self.pose.theta
                        angle_error = atan2(sin(target_angle - current_angle), cos(target_angle - current_angle))
                        angle_speed = angle_controller.update(-angle_error)
                        print("linear_speed:", linear_speed, "angle_speed:", angle_speed)
                        print("angle_error",angle_error)
                        self.control(self.vel_pub, linear_speed, angle_speed)
                        # print("linear_speed:",linear_speed)
                        # self.control(self.vel_pub, linear_speed, 0)

                elif self.mode == mode.turn:
                    print("TURN.............................................")
                    print("waypoint:",waypoint)
                    current_angle = self.pose.theta
                    target_angle = atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x)
                    print("target_angle:",target_angle)
                    angle_error = atan2(sin(target_angle - current_angle), cos(target_angle - current_angle))
                    print("angle_error:",angle_error)
                    if abs(angle_error) < angle_tolerance:
                        self.mode = mode.forward  

                        # if waypoints.index(waypoint) == len(waypoints) - 1:  
                        #     self.mode = mode.reach_goal
                        # else:
                        #     self.mode = mode.forward  
                        self.control(self.vel_pub, 0, 0) 
                        # self.mode = mode.reach_goal
                    else:
                        angle_speed = angle_controller.update(-angle_error)
                        self.control(self.vel_pub, 0, angle_speed)

                elif self.mode == mode.reach_goal:
                    rospy.loginfo("Reached final goal.")
                    break  

                self.rate.sleep()

    def control(self,pub, linear, angular):
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            pub.publish(cmd)
                
    
        # for waypoint in waypoints:

        #     reached_flag = False
        #     while not reached_flag and not rospy.is_shutdown():
        #         delta_x = waypoint[0] - self.pose.x
        #         delta_y = waypoint[1] - self.pose.y
        #         distance = sqrt(delta_x**2 + delta_y**2)
        #         target_angle = atan2(delta_y, delta_x)

        #         linear_speed = distance_controller.update(distance)

        #         angle_error = target_angle - self.pose.theta
        #         angle_error2 = atan2(sin(angle_error), cos(angle_error))

        #         angle_speed = angle_controller.update(angle_error2)

        #         if distance < distance_tolerance and abs(angle_error2) < angle_tolerance:
        #             reached_flag = True
        #             linear_speed = 0
        #             angle_speed = 0

        #         cmd_vel = Twist()
        #         cmd_vel.linear.x = linear_speed if abs(angle_error2) < angle_tolerance else 0
        #         cmd_vel.angular.z = angle_speed
        #         print("angle_speed:",angle_speed)
        #         print("angle_error:",angle_error2)
        #         # print("target_angle:",target_angle)
        #         self.vel_pub.publish(cmd_vel)

        #         self.rate.sleep()
            
        #     rospy.sleep(1)


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0, output_limits=(None,None)):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0
        self.output_min, self.output_max = output_limits

    def update(self, current_value):
        # calculate P_term and D_term
        # add your code here
        error = self.set_point - current_value
        P_term = self.Kp * error

        D_term = self.Kd * (error - self.previous_error)

        self.previous_error = error

        output = P_term + D_term

        if self.output_min is not None and output < self.output_min:
            output = self.output_min
        elif self.output_max is not None and output > self.output_max:
            output = self.output_max

        return output

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

    def setOutputLimits(self, min_value, max_value):
       
        self.output_min = min_value
        self.output_max = max_value


if __name__ == '__main__':
    whatever = Turtlebot()
