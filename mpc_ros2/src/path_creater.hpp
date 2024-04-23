#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <mutex>
#include <memory>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"

// MODE 0 : straight
//      1 : curve
#define MODE 1

class path_creater : public rclcpp::Node 
{
public:
    path_creater() : Node("path_creater")
    {}
    ~path_creater()
    {}
    void run();
    void init();
private:
    rclcpp::Clock ros2time;

    // setting parameter
    double dt = 0.2;
    int node = 30;
    double v_mean = 0.1;
    double x_max = 10.0;
    double rate = 10.0;
    double curve_y_max = 1.0;
    int global_node_max = 0;

    double f1_x = 0.0;
    double f1_y = -1.0;

    double f2_x = 0.0;
    double f2_y = 1.0;

    double f3_x = 0.0;
    double f3_y = -2.0;

    double f4_x = 0.0;
    double f4_y = 2.0;

    //ROS2
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lead_ref_traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr f1_ref_traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr f2_ref_traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr f3_ref_traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr f4_ref_traj_pub;

    int make_ref_id = 0;

    bool traj_end = false;

    tf2::Quaternion last_q;

    nav_msgs::msg::Path global_path_msg;
    nav_msgs::msg::Path lead_ref_traj_msg;
    nav_msgs::msg::Path f1_ref_traj_msg;
    nav_msgs::msg::Path f2_ref_traj_msg;
    nav_msgs::msg::Path f3_ref_traj_msg;
    nav_msgs::msg::Path f4_ref_traj_msg;

    void global_path();
    void make_lead_ref_trajectory();
    void make_lead_ref_theta();
    void make_follow_ref_trajectory();
    void publish();
    void reset();
    
};