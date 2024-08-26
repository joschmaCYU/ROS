#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

ros::Publisher cmd_vel_pub;
turtlesim::Pose current_pose;

double target_x;
double target_y;
double distance_tolerance = 0.1;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
    //ROS_INFO("Turtle Position -> x: [%f], y: [%f], theta: [%f]", current_pose.x, current_pose.y, current_pose.theta);
}

void moveToGoal()
{
    geometry_msgs::Twist cmd_vel_msg;

    double K_linear = 0.5;
    double K_angular = 6.0;

    float dx = target_x - current_pose.x;
    float dy = target_y - current_pose.y;

    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    ROS_INFO("Current angle [%f]", current_pose.theta * 180.0 / M_PI);
    
    double angle_to_goal = std::atan2(dy, dx);
    double angle_to_goal_degrees = angle_to_goal * 180.0 / M_PI;
    ROS_INFO("Angle to goal (degrees): [%f]", angle_to_goal_degrees);

    // Check if the robot is already at the target pose
    if (distance < 0.01) {
        // Robot is already at the target pose, no need to move
        return;
    }

    if (distance >= distance_tolerance) {
        cmd_vel_msg.angular.z = K_angular * (angle_to_goal - current_pose.theta);

        if ((std::abs(current_pose.theta * 180.0 / M_PI) - std::abs(angle_to_goal_degrees)) < 0.1) {
            cmd_vel_msg.linear.x = K_linear * distance;
        }

        //cmd_vel_msg.linear.x = K_linear * distance;
        
    } else {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        ROS_INFO("Turtle reached the goal!");
        ros::shutdown(); // Optionally shut down the node after reaching the goal
    }

    cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to");
    ros::NodeHandle nh;

    nh.param("target_x", target_x, 5.0);
    nh.param("target_y", target_y, 5.0);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        moveToGoal();
        loop_rate.sleep();
    }

    return 0;
}

