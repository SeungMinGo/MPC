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
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class global_path_creater : public rclcpp::Node
{
public:
    global_path_creater() : Node("global_path_creater")
    {}
    ~global_path_creater()
    {}

    void init();
    void run();
private:   

    rclcpp::Clock ros2time;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rbt1_odom_sub;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rbt1_global_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rbt1_ref_traj_pub;

    nav_msgs::msg::Path rbt1_global_path_msg;
    nav_msgs::msg::Path rbt1_ref_traj_msg;

    std::queue<nav_msgs::msg::Odometry> rbt1_odom_queue;
    std::mutex buf;

    bool rbt1_odom_in = false;
    bool traj_end = false;

    double dt = 0.2;
    int N = 30;

    double ref_traj_start = 0;
    double traj_make_time = 0;
    double global_x_max = 0;
    tf2::Quaternion last_q;

    double x_max = 20.0;
    double global_node_max = 50;
    double global_sampling = 10.0;

    void rbt1_odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void rbt1_odom_update();
    void global_path_create();
    void ref_traj_create();
    void ref_traj_angle_set();
    void global_path_msg_set();
    void ref_traj_msg_set();
    void publish();
    void reset();
};


