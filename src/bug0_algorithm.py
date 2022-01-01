#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt
from sensor_msgs.msg import LaserScan


class AvoidObstacles:

    def __init__(self):
        # set the goal and robot initial info
        self.xr = 16
        self.yr = 7.5

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.disthr = 0.0
        self.KpL = 0.7
        self.KpA = 3.0

        self.max_vel = 1.0
        self.set_vel = Twist()

        self.regions = {
            'front1':  10,
            'front2':  10,
            'right':  10,
            'left':  10
        }


    def bug0_algorithm(self):

        self.sub_odometry = rospy.Subscriber("odom/", Odometry,  self.callback_odometry_msg)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.obstacle_exists = False

        while self.get_euclidean_distance() > self.disthr:
            if self.obstacle_exists is False:
                self.go_to_goal()
            else:
                self.follow_wall()


    def go_to_goal(self):
        # set distance between robot and detected obstacle
        max_distance = 1
        rospy.loginfo('go to goal')

        # go to goal while no obstacle exists in the front of the robot
        while self.get_euclidean_distance() > self.disthr and self.obstacle_exists is False:

            # if any obstacle exists, set obstacle_exists to True and call follow_wall function
            if self.regions['front1'] < max_distance and self.regions['front2'] < max_distance:
                self.obstacle_exists = True

            # avoid known boxes
            self.avoid_boxes()

            # go to the goal
            self.set_vel.linear.x = self.KpL * min(self.get_euclidean_distance(), self.max_vel)
            self.set_vel.angular.z = self.KpA * (atan2(self.yr - self.y, self.xr - self.x) - self.theta)
            self.pub_vel.publish(self.set_vel)
            self.rate.sleep()

        # stop when the robot arrives the goal
        self.set_vel.linear.x = 0.0
        self.set_vel.angular.z = 0.0
        rospy.loginfo('The robot has arrived!')
        self.pub_vel.publish(self.set_vel)

    # follow wall function
    def follow_wall(self):
        # set distance between robot and detected obstacle
        max_distance = 1
        if self.regions['front1'] < max_distance and self.regions['front2'] < max_distance:
            self.turn_left()
        elif self.regions['front1'] < max_distance and self.regions['front2'] < max_distance and self.regions['right'] < max_distance:
            self.turn_left()
        elif self.regions['front1'] < max_distance and self.regions['front2'] < max_distance and self.regions['left'] < max_distance:
            self.turn_left()
        elif self.regions['front1'] > max_distance and self.regions['front2'] > max_distance and self.regions['left'] > max_distance and self.regions['right'] < max_distance:
            self.follow_the_wall()

        if self.regions['front1'] > max_distance and self.regions['front2'] > max_distance and self.regions['left'] > max_distance and self.regions['right'] > max_distance:
            self.obstacle_exists = False

        self.pub_vel.publish(self.set_vel)


    # avoid known obstacles
    def avoid_boxes(self):
        if self.x < 1.0 and self.y < 4.0:
            self.xr = 1.0
            self.yr = 4.5
        elif self.x < 8.1 and self.y < 6.5:
            self.xr = 8.5
            self.yr = 6.0
        else:
            self.xr = 16
            self.yr = 7.5


    # turn left function
    def turn_left(self):
        rospy.loginfo('turn left')
        self.set_vel.linear.x = 0
        self.set_vel.angular.z = 0.3

    # follow wall function
    def follow_the_wall(self):
        rospy.loginfo('follow the wall')
        self.set_vel.linear.x = 0.3
        self.set_vel.angular.z = 0

    # get euclidean distance function
    def get_euclidean_distance(self):
        return sqrt(pow((self.x - self.xr), 2) + pow((self.y - self.yr), 2))

    # get robot position
    def callback_odometry_msg(self, data):
        qtrn = data.pose.pose.orientation
        [roll, pitch, self.theta] = euler_from_quaternion(
            [qtrn.x, qtrn.y, qtrn.z, qtrn.w])

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    # get laser sensor regions messages
    def callback_laser(self, msg):
        self.regions = {
            'front1':  min(min(msg.ranges[330:359]), 10),
            'front2':  min(min(msg.ranges[0:29]), 10),
            'left':  min(min(msg.ranges[30:90]), 10),
            'right':  min(min(msg.ranges[260:329]), 10)
        }

if __name__ == '__main__':
    rospy.init_node("robot_control", anonymous=True)

    x = AvoidObstacles()
    x.bug0_algorithm()

    rospy.spin()
