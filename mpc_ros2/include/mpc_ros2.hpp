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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"

#include "casadi/casadi.hpp"
#include "tic_toc.hpp"

using std::placeholders::_1;

class mpc_ros2 : public rclcpp::Node
{
public:
    mpc_ros2() : Node("mpc_ros2")
    {}
    ~mpc_ros2()
    {}

    void init();
    void run();

private:
    // ROS2
    // ROS2 관련 변수들
    rclcpp::Clock ros2time; // ROS2 시간 관련 객체

    // ROS2 Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rbt1_odom_sub; // 로봇1의 오도메트리 정보를 구독하는 Subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rbt1_ref_traj_sub; // 로봇1의 참조 경로를 구독하는 Subscriber

    // ROS2 Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rbt1_pose_pub; // 로봇1의 위치를 발행하는 Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rbt1_cmd_pub; // 로봇1의 제어 명령을 발행하는 Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rbt1_pre_pub; // 로봇1의 예측 경로를 발행하는 Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub; // 장애물 정보를 발행하는 Publisher

    // ROS2 메시지 변수들
    nav_msgs::msg::Odometry rbt1_odom_msg; // 로봇1의 오도메트리 메시지
    nav_msgs::msg::Path rbt1_ref_traj_msg; // 로봇1의 참조 경로 메시지
    geometry_msgs::msg::PointStamped rbt1_pose_msg; // 로봇1의 위치 메시지
    geometry_msgs::msg::Twist rbt1_cmd_msg; // 로봇1의 제어 명령 메시지
    nav_msgs::msg::Path rbt1_pre_msg; // 로봇1의 예측 경로 메시지
    nav_msgs::msg::Path rbt1_ref_msg; // 로봇1의 참조 메시지
    visualization_msgs::msg::MarkerArray obs_msg; // 장애물 메시지

    // 오도메트리와 참조 경로를 저장하기 위한 큐와 뮤텍스
    std::queue<nav_msgs::msg::Odometry> rbt1_odom_queue; // 로봇1의 오도메트리 큐
    std::queue<nav_msgs::msg::Path> rbt1_ref_traj_queue; // 로봇1의 참조 경로 큐
    std::mutex buf; // 뮤텍스

    bool rbt1_odom_in = false; // 로봇1의 오도메트리 수신 여부
    bool rbt1_ref_traj_in = false; // 로봇1의 참조 경로 수신 여부

    // MPC 관련 변수들
    double dt = 0.1; // 시간 간격
    int N = 30; // 예측 스텝 수

    std::vector<std::vector<double>> obstacle_vec; // 장애물 벡터
    int n_obstacle; // 장애물 개수

    // 상태 변수 및 제어 변수
    casadi::SX x, y, theta, states; // 상태 변수
    int n_states; // 상태 변수 개수

    casadi::SX v, omega, controls; // 제어 변수
    int n_controls; // 제어 변수 개수

    casadi::SX rhs; // 미분 방정식 우변

    // 목적 함수 가중치
    double Q_x = 0.1; // X 위치 가중치
    double Q_y = 0.1; // Y 위치 가중치
    double Q_theta = 0.01; // 각도 가중치

    double R1 = 0.005; // 선속도 제어 입력 가중치
    double R2 = 0.005; // 각속도 제어 입력 가중치

    casadi::Function solver; // 최적화 문제를 해결하기 위한 솔버
    casadi::Function f; // 미분 방정식
    casadi::SX U, X, P; // 최적화 문제 변수 (U : 제어 변수, X: 상태 변수, P : 매개변수)

    casadi::SX Q, R; // 목적 함수 가중치

    casadi::SX g; // 제약 조건

    

    // 변수의 상한 및 하한
    double x_max = casadi::inf; // X 위치 상한
    double x_min = -casadi::inf; // X 위치 하한
    double y_max = casadi::inf; // Y 위치 상한
    double y_min = -casadi::inf; // Y 위치 하한
    double theta_max = casadi::inf; // 각도 상한
    double theta_min = -casadi::inf; // 각도 하한

    double v_max = 0.2; // 선속도 상한
    double v_min = 0.0; // 선속도 하한
    double omega_max = M_PI / 4.0; // 각속도 상한
    double omega_min = -(M_PI / 4.0); // 각속도 하한

    // 최적화 문제의 상한 및 하한
    // 장애물
    casadi::DM lbg; // 제약 조건 하한
    casadi::DM ubg; // 제약 조건 상한
    //상태
    casadi::DM lbx; // 변수 하한
    casadi::DM ubx; // 변수 상한

    casadi::DMDict args; // 최적화 문제 인자 딕셔너리

    casadi::DM rbt1_state_init; // 로봇1 초기 상태
    casadi::DM X0; // 초기 상태 변수
    casadi::DM U0; // 초기 제어 변수

    std::vector<std::vector<double>> ref_path; // 참조 경로

    casadi::DMDict solver_result; // 최적화 문제 결과

    int loop_count = 0; // 루프 카운터
    double traj_make_time = 0.0; // 경로 생성 시간

    // 잠재 함수 매개변수
    double gamma = 10.0; // 마지막 노드 가중치
    // 포텐셜 필드 관련 변수들 (지금은 사용 X)
    double P_ = 10.0; // P_
    double beta = 0.3; // 베타
    double weight_potential = 7.5; // 잠재 함수 가중치
    double a_ = 25; // a_
    double b_ = 20; // b_
    double c_ = 15; // c_

    // loop
    void rbt1_odom_Callbck(const nav_msgs::msg::Odometry::SharedPtr msg);
    void rbt1_ref_traj_Callbck(const nav_msgs::msg::Path::SharedPtr msg);
    void rbt1_odom_update();
    void rbt1_ref_traj_update();
    void state_update();
    void setting_solver();
    void make_args_p();
    void desired_trajectory();
    void setting_reference();
    void reshape_and_init_opt_variable();
    void call_solver();
    void get_result();
    void current_pos_msg_set();
    void cmd_vel_msg_set();
    void predictive_traj_msg_set();
    void ref_traj_msg_set();
    void obstacle_vis_msg_set();
    void publish();
    void shift();
    void reset();

    // once
    void mpc_init();
    void obstacle_set();

    //opt
    void collision_avoidance();
};

