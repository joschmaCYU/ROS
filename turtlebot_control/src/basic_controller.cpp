#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

class TurtleController {
private:
    ros::Publisher cmd_vel_pub;
    ros::NodeHandle private_nh; // Private node handle for private parameters
    double linear_speed;
    double angular_speed;

    geometry_msgs::Twist calculateCommand() {
        auto msg = geometry_msgs::Twist();

        // two new member
        msg.linear.x = linear_speed;
        msg.angular.z = angular_speed;

        return msg;
    }

    void getParameters() {
        // Get the parameters from the private node handle
        private_nh.param("linear_speed", linear_speed,0.0);  
        private_nh.param("angular_speed", angular_speed,0.0); 
    }

public:
    TurtleController() : private_nh("~") {  // Initialize the private node handle
        // Initialize ROS node handle
        ros::NodeHandle n;

        
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

        
        getParameters();

        // cout
        //std::cout << "Initialized with Linear Speed: " << linear_speed
        //          << ", Angular Speed: " << angular_speed << std::endl;
    }

    void run() {
        // Send messages in a loop
        ros::Rate loop_rate(10); // Loop rate of 10 Hz
        while (ros::ok()) {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            cmd_vel_pub.publish(msg);

            //cout
            //std::cout << "Publishing - Linear Speed: " << msg.linear.x
            //          << ", Angular Speed: " << msg.angular.z << std::endl;

            // Throttle the loop
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "turtle_controller");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.run();

    return 0;
}
