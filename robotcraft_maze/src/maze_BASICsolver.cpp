#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define STATE_LOST  0
#define STATE_FOLLOW_WALL 1

bool sensR = false;
bool sensL = false;
bool sensF = false;

bool sensRToNear = false;
bool sensLToNear = false;
bool sensFToNear = false;

double range_min = 80;
double range_max = 150;

bool stop_robot = false;  // Flag to control stopping the robot

double power = 1.0;
double low_power = 0.1;

// Publisher for movement commands
ros::Publisher cmd_vel_pub;

void sensR_Callback(const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data < range_min) {
        sensRToNear = true;
    } else {
        sensRToNear = false;
    }
    if (msg->data < range_max) {
        sensR = true;
    } else {
        sensR = false;
    }
}

void sensL_Callback(const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data < range_min) {
        sensLToNear = true;
    } else {
        sensLToNear = false;
    }

    if (msg->data < range_max) {
        sensL = true;
    } else {
        sensL = false;
    }
}

void sensF_Callback(const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data < range_min) {
        sensFToNear = true;
    } else {
        sensFToNear = false;
    }

    if (msg->data < range_max) {
        sensF = true;
    } else {
        sensF = false;
    }
}

void stop_Callback(const std_msgs::Int32::ConstPtr& msg) {
    if (msg->data == 0) {
        stop_robot = true;
    } else {
        stop_robot = false;
    }
}

void move(double linear_speed, double angular_speed) {
    if (!stop_robot) {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_speed;
        twist.angular.z = angular_speed;
        cmd_vel_pub.publish(twist);
    } else {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmd_vel_pub.publish(twist);
    }
}

int lost() {
    if (!sensR && !sensL && !sensF) {
        move(power, 0.0); // Move forward
    } else {
        return STATE_FOLLOW_WALL;
    }
    return STATE_LOST;
}

int follow_wall() {
    if (sensF) {
        move(0.0, power); // Turn left if obstacle in front
    } else if (sensRToNear) {
        move(low_power, power); // Turn left if too close to right wall
    } else if (!sensR) {
        move(low_power, -power); // Turn right if too far from right wall
    } else {
        move(power, 0.0); // Move forward
    }
    return STATE_FOLLOW_WALL;
}

void ros_okay(ros::Rate rate, int state) {
    while (ros::ok()) {
        ROS_INFO("State: %d, SensL: %d, SensR: %d, SensF: %d", state, sensL, sensR, sensF);
        switch(state) {
            case STATE_LOST:
                state = lost();
                break;
            case STATE_FOLLOW_WALL:
                state = follow_wall();
                break;
            default:
                state = STATE_LOST;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "maze_solver");
    ros::NodeHandle nh;

    // Initialize the publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Create subscribers for the sensor callbacks
    ros::Subscriber sub_sensR = nh.subscribe("right_distance", 10, sensR_Callback);
    ros::Subscriber sub_sensL = nh.subscribe("left_distance", 10, sensL_Callback);
    ros::Subscriber sub_sensF = nh.subscribe("front_distance", 10, sensF_Callback);

    // Create subscriber for the stop command
    ros::Subscriber sub_stop = nh.subscribe("stop_cmd", 10, stop_Callback);

    int state = STATE_LOST;
    ros::Rate rate(10);

    ros_okay(rate, state);

    return 0;
}