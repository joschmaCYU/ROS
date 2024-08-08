#!/bin/python3

#/base_pose_ground_truth
# rostopic type : nav_msgs/Odometry
# rostopic echo : 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation():
    def __init__(self):
        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()
        self.desired_distance_from_wall = 1.0
        self.max_distance = 3.0
        self.side = 'left' # change this to make the robot follow the choosen wall
        self.obstacle_distance = 100
        self.rospy_sub_lazer = rospy.Subscriber("base_scan", LaserScan, self.laser_callback, queue_size=1)
        self.pub_CMD = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_callback(self, callback):
        self.laser_msg = callback

    def calculate_command(self):
        if type(self.laser_msg.ranges) == tuple:
            side_distances = self.laser_msg.ranges
            num_ranges = len(side_distances)
            half_length = num_ranges / 2

            left_side_distances = []
            right_side_distances = []
            
            for i in range(num_ranges):
                if i < half_length:
                    left_side_distances.append(side_distances[i])
                else:
                    right_side_distances.append(side_distances[i])

            if len(side_distances) > 0:
                self.obstacle_distance = min(side_distances)

                if self.obstacle_distance < self.desired_distance_from_wall: # To far
                    self.cmd_vel.linear.x = 0.5
                    if self.side == 'left':
                        self.cmd_vel.angular.z = -1.0
                    else:
                        self.cmd_vel.angular.z = 1.0
                elif self.obstacle_distance > self.desired_distance_from_wall: # To close
                    self.cmd_vel.linear.x = 0.5
                    if self.side == 'left':
                        self.cmd_vel.angular.z = 1.0
                    else:
                        self.cmd_vel.angular.z = -1.0
                else: # Correct distance
                    self.cmd_vel.linear.x = 1.0
                    self.cmd_vel.angular.z = 0.0
            else:
                self.cmd_vel.linear.x = 1.0
                self.cmd_vel.angular.z = 0.0

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.pub_CMD.publish(self.cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("reactive_navigation_py")
    controller = ReactiveNavigation()
    controller.run()
