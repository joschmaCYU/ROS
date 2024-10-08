#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <queue>
#include <iostream>
#include <vector>
#include <fstream>
#include <limits>
#include <cmath>
#include <utility>
#include <sstream>

// robotcraft2019_robot(pose [ 0.5  5.0  0.000  0.000 ] name "robotcraft_robot_groupX")

using namespace std;
ros::Publisher cmd_vel_pub;
double robot_x = 0.0;
double robot_y = 0.0;
double theta = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // pose [ 0.5  5.0  0.000  0.000 ]
    robot_x = msg->pose.pose.position.x + 5;
    robot_y = msg->pose.pose.position.y + 5;

    tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    theta = tf::getYaw(quat);
}

int argmin(const vector<vector<int>>& M, int target, bool isRow, int index) {
    double min_diff = numeric_limits<double>::max();
    int min_index = -1;

    if (isRow) {
        for (int j = 0; j < M[0].size(); ++j) {
            double diff = abs(M[index][j] - target);
            if (diff < min_diff) {
                min_diff = diff;
                min_index = j;
            }
        }
    } else {
        for (int i = 0; i < M.size(); ++i) {
            double diff = abs(M[i][index] - target);
            if (diff < min_diff) {
                min_diff = diff;
                min_index = i;
            }
        }
    }

    return abs(min_diff);
}

vector<vector<int>> get_matrix(string filePath) {
    string filename = filePath;
    ifstream file(filename);
    vector<vector<int>> matrix;
    
    if (!file.is_open()) {
        cerr << "Could not open the file " << filename << endl;
        return matrix;
    }
    
    string line;

    while (getline(file, line)) {
        vector<int> row;
        stringstream ss(line);
        int value;

        while (ss >> value) {
            row.push_back(value);
        }

        matrix.push_back(row);
    }

    file.close();
    return matrix;
}

void ros_okay(ros::Rate rate) {
    while (ros::ok()) { 
        ros::spinOnce();
        rate.sleep();
    }
}

void calculateDifferences(const vector<pair<int, int>>& path, vector<pair<double, double>>& differences) {
    if (path.empty()) return;

    double prev_i = path[0].first;
    double prev_j = path[0].second;

    for (size_t k = 1; k < path.size(); ++k) {
        double current_i = path[k].first;
        double current_j = path[k].second;

        double diff_i = current_i - prev_i;
        double diff_j = current_j - prev_j;

        differences.push_back(make_pair(diff_i, diff_j));

        prev_i = current_i;
        prev_j = current_j;
    }
}

void move_robot_to_target(vector<pair<int, int>>& path, int xstart, int ystart) {
    geometry_msgs::Twist move_cmd;
    // for x vector same as robot
    // y inversed for robot and vector

    double target_i = xstart;
    double target_j = ystart;

    path.insert(path.begin(), std::make_pair(xstart, ystart));

    vector<pair<double, double>> differences;
    calculateDifferences(path, differences);

    int k = 0;
    for (const auto diff : differences) {
        target_i += diff.second * 0.1;
        target_j -= diff.first * 0.1;

        while (ros::ok()) {
            double irobot = robot_x;
            double jrobot = robot_y;
            float dx = target_i - irobot;
            float dy = target_j - jrobot;
            ROS_WARN("Robot's current position: (%f, %f)", irobot, jrobot);
            ROS_WARN("targ target: (%f, %f)", target_i, target_j);
            ROS_WARN("Path %d %d", path[k].first, path[k].second);

            double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            double angle_to_goal = std::atan2(dy, dx);
            double angle_to_goal_degrees = angle_to_goal * 180.0 / M_PI;

            float linear_vel = 0.0;
            float angular_vel = 0.0;

            float distance_tolerance = 0.2;
            float angular_tolerance = 0.2;

            double angular_error = angle_to_goal - theta;

            if (abs(angular_error) >= angular_tolerance) {
                angular_vel = angular_error;
                linear_vel = 0.0;
            } else if (abs(distance) >= distance_tolerance) {
                linear_vel = distance;
            } else {
                linear_vel = 0.0;
                angular_vel = 0.0;
                break;
            }

            linear_vel *= 0.5;
            angular_vel *= 0.3;

            move_cmd.linear.x = linear_vel;
            move_cmd.angular.z = angular_vel;

            cmd_vel_pub.publish(move_cmd);

            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        k++;
    }
}

void save_matrix(const vector<vector<int>>& matrix, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Could not open the file " << filename << endl;
        return;
    }

    for (const auto& row : matrix) {
        for (const auto& val : row) {
            file << val << " ";
        }
        file << endl;
    }

    file.close();
}

void save_path(const vector<pair<int, int>>& path, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Could not open the file " << filename << endl;
        return;
    }

    for (const auto& point : path) {
        file << point.first << " " << point.second << endl;
    }

    file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "maze_PROsolver");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    vector<vector<int>> M = get_matrix("/home/josch/catkin_ws/src/robotcraft_maze/world/grid_matrix.txt");

    int xtarget = 50, ytarget = 5;
    int xstart = 5, ystart = 5;
    
    int n = M.size();
    int m = M[0].size();
    
    int itarget = argmin(M, xtarget, true, 0);
    int jtarget = argmin(M, ytarget, false, 0);
    int istart = argmin(M, xstart, true, 0);
    int jstart = argmin(M, ystart, false, 0);

    vector<vector<int>> A = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };
    vector<vector<int>> C(n, vector<int>(m, 0));
    
    C[itarget][jtarget] = 1;
    
    queue<pair<int, int>> O;
    O.push({itarget, jtarget});
    
    while (!O.empty()) {
        pair<int, int> current = O.front();
        O.pop();
        
        for (const auto& r : A) {
            int aux_i = current.first + r[0];
            int aux_j = current.second + r[1];

            //𝑖𝑓 1 ≤ 𝑎𝑢𝑥 1,1 ≤ dim 𝑥𝑎𝑥𝑖𝑠 𝑎𝑛𝑑 1 ≤ 𝑎𝑢𝑥 1,2 ≤ dim 𝑦𝑎𝑥𝑖𝑠 𝑎𝑛𝑑 𝑀 𝑎𝑢𝑥 = 0 𝑎𝑛𝑑 𝐶 𝑎𝑢𝑥 = 0
            if (0 < aux_i && aux_i < n && 0 < aux_j && aux_j < m && C[aux_i][aux_j] == 0 && M[aux_i][aux_j] == 0) {
                C[aux_i][aux_j] = C[current.first][current.second] + 1;
                O.push({aux_i, aux_j});
            }
        }
    }
    
    vector<pair<int, int>> P;
    P.push_back({istart, jstart});
    
    while (P.back() != make_pair(itarget, jtarget)) {
        // This retrieves the cost at the current position
        int aux_c = C[P.back().first][P.back().second];
        
        for (const auto& r : A) {
            int aux_i = P.back().first + r[0];
            int aux_j = P.back().second + r[1];
            
            //𝑖𝑓 1 ≤ 𝑎𝑢𝑥 1,1 ≤ dim 𝑥𝑎𝑥𝑖𝑠 𝑎𝑛𝑑  1 ≤ 𝑎𝑢𝑥 1,2 ≤ dim 𝑦𝑎𝑥𝑖𝑠  𝑎𝑛𝑑 𝐶 𝑎𝑢𝑥 ≠ 0
            if (0 < aux_i && aux_i < n && 0 < aux_j && aux_j < m && C[aux_i][aux_j] != 0) { 
                if (C[aux_i][aux_j] < aux_c) {
                    P.push_back({aux_i, aux_j});
                    break;
                }
            }
        }
    }

    vector<vector<int>> out = get_matrix("/home/josch/catkin_ws/src/robotcraft_maze/world/grid_matrix.txt");

    for (int i = 0; i < P.size(); ++i) {
        for (int j = 0; j < P.size(); ++j) {
            for (int k = 0; k < P.size(); ++k) {
                if (P[i].first == j && P[i].second == k) {
                    //ROS_INFO("P: %d %d / out: %d %d", P[i].first, P[i].second, j, k);
                    out[j][k] = 2;
                }
            }
        }
    }

    out[istart][jstart] = 3;
    out[itarget][jtarget] = 4;

    save_matrix(out, "/home/josch/catkin_ws/src/robotcraft_maze/world/out.txt");

    save_matrix(C, "/home/josch/catkin_ws/src/robotcraft_maze/world/C_matrix.txt");
    save_path(P, "/home/josch/catkin_ws/src/robotcraft_maze/world/P_path.txt");
    
    move_robot_to_target(P, xstart, ystart);
    
    return 0;
}