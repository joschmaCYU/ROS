#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include "std_msgs/UInt8MultiArray.h"

float front_distance = 1.0;
float right_distance = 1.0;
float left_distance = 1.0;
float kd_value = 0.0;

void sensorFrontCallback(const std_msgs::Float32& msg) {
    front_distance = msg.data;
}

void sensorRightCallback(const std_msgs::Float32& msg) {
    right_distance = msg.data;
}

void sensorLeftCallback(const std_msgs::Float32& msg) {
    left_distance = msg.data;
}

geometry_msgs::Pose2D pose;

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    pose = *msg;
}

void checkCollision() {
    if (front_distance < 150) {
        ROS_WARN("Collision detected in front");
    }
    if (right_distance < 150) {
        ROS_WARN("Collision detected in right");
    }
    if (left_distance < 150) {
        ROS_WARN("Collision detected in left");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher rgb_leds_pub = nh.advertise<std_msgs::UInt8MultiArray>("/rgb_leds", 10);
    ros::Publisher go_to_pub = nh.advertise<geometry_msgs::Pose2D>("/set_pose", 10);

    std_msgs::UInt8MultiArray rgb_msg;
    rgb_msg.data = {255, 0, 0};
    rgb_leds_pub.publish(rgb_msg);

    ros::Subscriber sub_front = nh.subscribe("/front_distance", 10, sensorFrontCallback);
    ros::Subscriber sub_right = nh.subscribe("/right_distance", 10, sensorRightCallback);
    ros::Subscriber sub_left = nh.subscribe("/left_distance", 10, sensorLeftCallback);
    ros::Subscriber sub_pose = nh.subscribe("/pose", 10, poseCallback);

    ros::Rate loop_rate(10);

    ros::Time start_time = ros::Time::now();
    int step = 0;

    while (ros::ok()) {
        checkCollision();
        
        geometry_msgs::Pose2D goal;
        float postion_tolerance = 0.1;

        ROS_INFO("%d, %d", std::abs(pose.x - goal.x) <= postion_tolerance, std::abs(pose.y - goal.y) <= postion_tolerance);
        ROS_INFO("Pose [%f, %f, %f], step: %i: ", pose.x, pose.y, pose.theta * 180.0 / M_PI, step);

        if (step == 0) {
            goal.x = 0.5;
            goal.y = 0.0;
        } else if (step == 1) {
            goal.x = 0.5;
            goal.y = 0.5;
        } else if (step == 2) {
            goal.x = 0.0;
            goal.y = 0.5;
        } else if (step == 3) {
            goal.x = 0.0;
            goal.y = 0.0;
        }

        if (std::abs(pose.x - goal.x) <= postion_tolerance && std::abs(pose.y - goal.y) <= postion_tolerance) {
            if (step == 3) {
                step = 0;
            } else {
                step++;
            }
        }

        go_to_pub.publish(goal);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}