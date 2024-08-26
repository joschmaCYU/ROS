#include "ros/ros.h"
#include "service/Status.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

bool showStatus(service::Status::Request  &req,
                service::Status::Response &res) {
  res.result = "ON";
  ROS_INFO("Request: value= %f", req.value);
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x = req.value;  // Using the request value as linear velocit
    msg.angular.z = 0.0;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return true;
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("request_service", showStatus);
  ROS_INFO("Ready to process service requests.");
  ros::spin();
  return 0;
}

