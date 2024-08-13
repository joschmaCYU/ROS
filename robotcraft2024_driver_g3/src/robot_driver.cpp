#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

#include <tf/transform_broadcaster.h>

ros::Publisher ir_front_pub;
ros::Publisher ir_right_pub;
ros::Publisher ir_left_pub;

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
    // Create a new odometry message
    nav_msgs::Odometry odom_msg;

    // Copy the pose information from the received message
    odom_msg.pose.pose = *pose_msg;

    // Set the frame ID to "odom"
    odom_msg.header.frame_id = "odom";

    // Set the child frame ID to "base_link"
    odom_msg.child_frame_id = "base_link";

    // Publish the odometry message on the "/odom" topic
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    odom_pub.publish(odom_msg);

    // Create a TransformBroadcaster
    static tf::TransformBroadcaster br;

    // Create a Transform message
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_msg->position.x, pose_msg->position.y, pose_msg->position.z));
    transform.setRotation(tf::Quaternion(pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z, pose_msg->orientation.w));

    // Broadcast the transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

void frontDistanceCallback(const std_msgs::Float32::ConstPtr& front_sensor)
{
    sensor_msgs::Range rangeMsg;
    rangeMsg.range = front_sensor->data;
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/front_ir";
	rangeMsg.radiation_type = 1;
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;

    ros::NodeHandle nh;
    ir_front_pub = nh.advertise<sensor_msgs::Range>("/sensor_front_ir", 10);
    ir_front_pub.publish(rangeMsg);
}

void rightDistanceCallback(const std_msgs::Float32::ConstPtr& right_sensor)
{
    sensor_msgs::Range rangeMsg;
    rangeMsg.range = right_sensor->data;
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/right_ir";
	rangeMsg.radiation_type = 1;
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;

    ros::NodeHandle nh;
    ir_right_pub = nh.advertise<sensor_msgs::Range>("/sensor_right_ir", 10);
    ir_right_pub.publish(rangeMsg);
}

void leftDistanceCallback(const std_msgs::Float32::ConstPtr& left_sensor)
{
    sensor_msgs::Range rangeMsg;
    rangeMsg.range = left_sensor->data;
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/left_ir";
	rangeMsg.radiation_type = 1;
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;

    ros::NodeHandle nh;
    ir_left_pub = nh.advertise<sensor_msgs::Range>("/sensor_left_ir", 10);
    ir_left_pub.publish(rangeMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber pose_sub = nh.subscribe("/pose", 10, poseCallback);

    ros::Subscriber front_distance_sub = nh.subscribe("/front_distance", 10, frontDistanceCallback);
    ros::Subscriber right_distance_sub = nh.subscribe("/right_distance", 10, rightDistanceCallback);
    ros::Subscriber left_distance_sub = nh.subscribe("/left_distance", 10, leftDistanceCallback);

    ros::spin();
    loop_rate.sleep();

    return 0;
}