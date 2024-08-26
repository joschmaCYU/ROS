#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define STATE_LOST  0
#define STATE_CCW   1
#define STATE_WALL1 2
#define STATE_WALL2 3

bool sensR = false;
bool sensL = false;
bool sensF = false;

bool sensRToNear = false;
bool sensLToNear = false;
bool sensFToNear = false;

ros::Publisher cmd_vel_pub;

void sensR_Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges[0] < msg->range_min *2) {
        sensRToNear = true;
    } else {
        sensRToNear = false;
    }
    if (msg->ranges[0] < msg->range_max/2) {
        sensR = true;
    } else {
        sensR = false;
    }
}

void sensL_Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges[0] < msg->range_min*2) {
        sensLToNear = true;
    } else {
        sensLToNear = false;
    }

    if (msg->ranges[0] < msg->range_max/2) {
        sensL = true;
    } else {
        sensL = false;
    }
}

void sensF_Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges[0] < msg->range_min*2) {
        sensFToNear = true;
    } else {
        sensFToNear = false;
    }

    if (msg->ranges[0] < msg->range_max/2) {
        sensF = true;
    } else {
        sensF = false;
    }
}


void move(double linear_speed) {
    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed;

    cmd_vel_pub.publish(twist);
}

void turn(double angular_speed) {
    geometry_msgs::Twist twist;
    twist.angular.z = angular_speed;

    cmd_vel_pub.publish(twist);
}

int lost() {
    if (!sensR && !sensL) { // && !sensF
    	move(0.3); //linear > 0 = move forward
    }else {
        return STATE_CCW;
    }
    return STATE_LOST;
}

int ccw() {

    if (sensL || sensR) {
        turn(0.3); //angular > 0 = turn left
    } else {
        return STATE_WALL1;
    }
    return STATE_CCW;
}

int wall1() {
    if (!sensR) {
        move(0.4);
        turn(-0.3); //angular < 0 = turn right, marche bien quand égal à 1
    } else {
        return STATE_WALL2;
    }
    return STATE_WALL1;
}

int wall2() {
    if (!sensL){
        if (sensR){
            if (sensRToNear) {
                turn(-0.5);
            } else {
                turn(0.5);
            }
            
            move(0.4);
        } else {
            return STATE_WALL1;
        }
    } else {
        return STATE_CCW;
    }
    return STATE_WALL2;
}

void ros_okay(ros::Rate rate, int state) {
    while (ros::ok()) { 
        switch(state) {
            case STATE_LOST: 
                ROS_INFO("Lost"); 
                state = lost(); 
                break;
            case STATE_CCW:
                ROS_INFO ("CCW"); 
                state = ccw(); 
                break;
            case STATE_WALL1 : 
                ROS_INFO ("Wall1"); 
                state = wall1(); 
                break;
            case STATE_WALL2: 
                ROS_INFO ("Wall2"); 
                state = wall2(); 
                break;
            default:
                ROS_ERROR("Invalid State");
                state = STATE_LOST;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "maze_BASICsolver");
    ros::NodeHandle nh;
    
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sensR_sub = nh.subscribe("base_scan_1", 10, sensF_Callback);
    ros::Subscriber sensL_sub = nh.subscribe("base_scan_2", 10, sensL_Callback);
    ros::Subscriber sensF_sub = nh.subscribe("base_scan_3", 10, sensR_Callback);

    int state = STATE_LOST;
    ros::Rate rate(10);

    ros_okay(rate, state);

    //MazeBASICSolver solver;
    //solver.run();

    return 0;
}