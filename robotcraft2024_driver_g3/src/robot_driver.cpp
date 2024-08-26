#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

ros::Publisher odom_pub;
ros::Publisher ir_front_sensor_pub;
ros::Publisher ir_right_sensor_pub;
ros::Publisher ir_left_sensor_pub;

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";  
    odom_msg.child_frame_id = "base_link";  
    
    odom_msg.pose.pose.position.x = pose_msg->x;
    odom_msg.pose.pose.position.y = pose_msg->y;
    odom_msg.pose.pose.orientation.z = sin(pose_msg->theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(pose_msg->theta / 2.0);

    odom_pub.publish(odom_msg);

    transform.setOrigin(tf::Vector3(pose_msg->x, pose_msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, pose_msg->theta);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

void frontCallback(const std_msgs::Float32::ConstPtr& front_distance_msg) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "front_ir"; 
    range_msg.radiation_type = 1;
    range_msg.field_of_view = 0.034906585;
    range_msg.min_range = 0.1;
    range_msg.max_range = 0.8;
    range_msg.range = front_distance_msg->data;

    ir_front_sensor_pub.publish(range_msg);
}

void rightCallback(const std_msgs::Float32::ConstPtr& right_distance_msg) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "right_ir"; 
    range_msg.radiation_type = 1;
    range_msg.field_of_view = 0.034906585;
    range_msg.min_range = 0.1;
    range_msg.max_range = 0.8;
    range_msg.range = right_distance_msg->data;

    ir_right_sensor_pub.publish(range_msg);
}

void leftCallback(const std_msgs::Float32::ConstPtr& left_distance_msg) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "left_ir";
    range_msg.radiation_type = 1;
    range_msg.field_of_view = 0.034906585;
    range_msg.min_range = 0.1;
    range_msg.max_range = 0.8;
    range_msg.range = left_distance_msg->data;

    ir_left_sensor_pub.publish(range_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    ir_front_sensor_pub = nh.advertise<sensor_msgs::Range>("/ir_front_sensor", 10);
    ir_right_sensor_pub = nh.advertise<sensor_msgs::Range>("/ir_right_sensor", 10);
    ir_left_sensor_pub = nh.advertise<sensor_msgs::Range>("/ir_left_sensor", 10);

    ros::Subscriber pose_sub = nh.subscribe("/pose", 10, poseCallback);
    ros::Subscriber front_distance_sub = nh.subscribe("/front_distance", 10, frontCallback);
    ros::Subscriber right_distance_sub = nh.subscribe("/right_distance", 10, rightCallback);
    ros::Subscriber left_distance_sub = nh.subscribe("/left_distance", 10, leftCallback);
    
    ros::spin();

    return 0;
}