#!/usr/bin/python

import rospy
import math
import random
import geometry_msgs.msg
from turtlesim.srv import Spawn
from mas_framework.msg import base
from mas_framework.msg import status
import turtlesim.msg as turtle


class Turtle:

    def __init__(self, number):

        self.number = number
        self.name = 'turtle'+str(self.number)
        self.status = "wait"
        self.base_msg = base()
        self.geometry_msg = geometry_msgs.msg.Twist()
        self.subscriber_pose = rospy.Subscriber(str(
            self.name)+'/pose', turtle.Pose, self.callback_pose, queue_size=1)
        self.velocity_publisher = rospy.Publisher("/turtle"+str(
            self.number)+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.coordinate_publisher = rospy.Publisher(
            "/environment", base, queue_size=1)
        self.action_publisher = rospy.Publisher("/turtle"+str(
            self.number)+"/action", base, queue_size=1)
        self.subscriber_coordinate = rospy.Subscriber(
            "/environment", base, self.callback_coordinate, queue_size=10)
        self.subscriber_status_msg = rospy.Subscriber(
            "/status", status, self.callback_status, queue_size=1)
        self.general_goal_name = "turtle1"
        self.local_goal_name = None
        self.x = 0
        self.y = 0
        self.phi = 0
        self.linear_velocity = 1
        self.angular_velocity = 1
        self.turtles = {self.name: [self.x, self.y, self.phi]}
        self.borders_points = {"first_point": [0, 10], "second_point": [10, 0]}

    def start(self):

        while self.general_goal_name not in self.turtles:
            rospy.sleep(3)

        if self.name != self.general_goal_name:
            self.trade()

	self.status = "move"
        while not rospy.is_shutdown():
	    
	    if self.name != self.general_goal_name:

                if self.status == "move":

                    if self.detect_border():
                        self.border()
			rospy.loginfo("border")
                    self.move()

    def move(self):

        x_goal = self.turtles[self.local_goal_name][0]
        y_goal = self.turtles[self.local_goal_name][1]

        if self.phi > math.pi:
            self.phi = self.phi - 2*math.pi

        if self.phi < -math.pi:
            self.phi = self.phi + 2*math.pi

        phi = self.phi
        theta = self.get_angle(y_goal - self.y, x_goal - self.x)
        distance = self.get_distance(x_goal, y_goal, self.x, self.y)
        angular = self.angular_velocity
        linear = self.linear_velocity

        if distance > 1:

            angular, linear = self.get_velocity(phi, theta)
	    self.geometry_msg.angular.z = angular
            self.geometry_msg.linear.x = linear

        else:

            self.geometry_msg.angular.z = 0
            self.geometry_msg.linear.x = 0

        self.velocity_publisher.publish(self.geometry_msg)

    def detect_border(self):

        for point in self.borders_points:
            if abs(self.x-self.borders_points[point][0]) < 1:
                return True

        for point in self.borders_points:
            if abs(self.y - self.borders_points[point][1]) < 1:
                return True

    def border(self):
        
        while self.detect_border():

                if self.phi > math.pi:
                    self.phi = self.phi - 2*math.pi

                if self.phi < -math.pi:
                    self.phi = self.phi + 2*math.pi

                phi = self.phi
		theta =  self.get_angle(5 - self.y, 5 - self.x)
                delta = phi - theta
                angular, linear = self.get_velocity(phi, theta)
	        self.geometry_msg.angular.z = angular
                self.geometry_msg.linear.x = linear

                self.velocity_publisher.publish(self.geometry_msg)

    def callback_pose(self, data):

        self.x = data.x
        self.y = data.y
        self.phi = data.theta
        self.base_msg.x = data.x
        self.base_msg.y = data.y
        self.base_msg.theta = data.theta
        self.base_msg.load = self.name
        self.coordinate_publisher.publish(self.base_msg)

    def callback_coordinate(self, data):

        self.turtles[data.load] = [data.x, data.y, data.theta]

    def trade(self):

        turtles = (sorted(self.turtles.items(), key=lambda item: (
            item[1][0] - self.turtles[self.general_goal_name][0]) ** 2 + (
                item[1][1] - self.turtles[self.general_goal_name][1]) ** 2))

        turtles_order = dict(
                [(turtles[i][0], i) for i in range(0, len(self.turtles))])

        inv_turtles_order = {v: k for k, v in turtles_order.items()}
        self.local_goal_name = inv_turtles_order[turtles_order[self.name]-1]

    def callback_status(self, data):

        if self.name != self.general_goal_name:
                self.trade()
        self.status = data.status
        rospy.sleep(1)

    def get_distance(self, x1, y1, x2, y2):

        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_angle(self, y, x):

        return math.atan2(y, x)

    def get_velocity(self, phi, theta):
        
        delta = phi - theta
        linear = self.linear_velocity
        angular = self.angular_velocity

        if abs(delta) > 0.1:

            if phi > 0:

                if theta > 0:

                    if delta > 0:
                        return -angular, linear

                    else:
                        return angular, linear

                else:

                    if delta > math.pi:
                        return angular, linear

                    else:
                        return -angular, linear

            else:

                if theta > 0:

                    if delta > -math.pi:
                        return angular, linear

                    else:
                        return -angular, linear

                else:

                    if delta > 0:
                        return -angular, linear

                    else:
                        return angular, linear

        else:
            return 0, linear


if __name__ == "__main__":

    rospy.init_node("~turtle", anonymous=True)
    number = rospy.get_param("~number")
    rospy.wait_for_service('spawn')

    if number != 1:

        x = rospy.get_param("~x_coordinate")
        y = rospy.get_param("~y_coordinate")
        theta = rospy.get_param("~theta_coordinate")
        spawn_turtle_x = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle_x(x, y, theta, '')

    x = Turtle(number)
    x.start()
    rospy.spin()
