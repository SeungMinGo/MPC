#include "new_mpc3.hpp"

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
    angle -= 2.*M_PI;

  while(angle < -M_PI)
    angle += 2.*M_PI;

  return angle;
}

double q_to_e(nav_msgs::msg::Odometry msg)
{
    tf2::Quaternion q;
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    return yaw;
}

double q_to_e_path(const geometry_msgs::msg::Pose& pose)
{
    tf2::Quaternion q;
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    return yaw;
}

tf2::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

void new_mpc3::init()
{
    rbt1_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",1,std::bind(&new_mpc3::rbt1_odom_Callbck,this,_1));
    rbt1_ref_traj_sub = this->create_subscription<nav_msgs::msg::Path>("/rbt1/ref_trajectory",1,std::bind(&new_mpc3::rbt1_ref_traj_Callbck,this,_1));
    rbt1_pose_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/rbt1/pose",1);
    rbt1_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    rbt1_pre_pub = this->create_publisher<nav_msgs::msg::Path>("/rbt1/pre_path",1);
}

void new_mpc3::rbt1_odom_Callbck(const nav_msgs::msg::Odometry::SharedPtr msg){
    buf.lock();
    rbt1_odom_queue.push(*msg);
    buf.unlock();
}

void new_mpc3::rbt1_ref_traj_Callbck(const nav_msgs::msg::Path::SharedPtr msg){
    buf.lock();
    rbt1_ref_traj_queue.push(*msg);
    buf.unlock();
}

void new_mpc3::rbt1_odom_update()
{
    if(!rbt1_odom_queue.empty())
    {
        buf.lock();
        rbt1_odom_msg = rbt1_odom_queue.front();
        rbt1_odom_queue.pop();
        rbt1_odom_in = true;
        buf.unlock();
    }
}

void new_mpc3::state_update()
{
    double rbt1_yaw = pi_to_pi(q_to_e(rbt1_odom_msg));
    rbt1_state_init = casadi::DM::vertcat({rbt1_odom_msg.pose.pose.position.x, rbt1_odom_msg.pose.pose.position.y, rbt1_yaw});
    // rbt1_state_init = casadi::DM::vertcat({1.0, 0.5, 0.002});
}

void new_mpc3::rbt1_ref_traj_update()
{
    if(!rbt1_ref_traj_queue.empty())
    {
        buf.lock();
        rbt1_ref_traj_msg = rbt1_ref_traj_queue.front();
        rbt1_ref_traj_queue.pop();
        rbt1_ref_traj_in = true;
        buf.unlock();
    }
}
// args에 들어가는 요소 중 "p"에 대한 요소 생성
void new_mpc3::make_args_p(){
    // 현재 상태 변수 + 노드 수 * (상태 변수 + 제어 변수)
    casadi::DM p = casadi::DM::zeros(n_states + N * (n_states + n_controls), 1);
    args.insert(std::pair<std::string, casadi::DM>("p", p));
    // 현재 상태 변수를 args "p"요소의 0,1,2 인덱스에 먼저 추가함
    for (int idx=0; idx<n_states; ++idx){
        args["p"](idx) = rbt1_state_init(idx);
    }
}

void new_mpc3::setting_reference(){
    for(int idx = 0; idx < N; ++idx){
        double theta_ref = 0.0;
        double u_ref = 0.5 / 6.0;
        double omega_ref = 0.0;

        theta_ref = q_to_e_path(rbt1_ref_traj_msg.poses[idx].pose);

        args["p"]((n_states + n_controls) * idx + 3) = rbt1_ref_traj_msg.poses[idx].pose.position.x;
        args["p"]((n_states + n_controls) * idx + 4) = rbt1_ref_traj_msg.poses[idx].pose.position.y;
        args["p"]((n_states + n_controls) * idx + 5) = theta_ref;
        args["p"]((n_states + n_controls) * idx + 6) = u_ref;
        args["p"]((n_states + n_controls) * idx + 7) = omega_ref;
    }
}

void new_mpc3::reshape_and_init_opt_variable()
{
    args.insert(std::pair<std::string,casadi::DM>("x0",casadi::DM::vertcat({casadi::DM::reshape(X0,n_states * (N+1),1) ,
                                                  casadi::DM::reshape(U0,n_controls * N,1)})));
}

// 얘는 맨 처음 한 번만 실행됨.
void new_mpc3::mpc_init()
{
    x      = casadi::SX::sym("x");
    y      = casadi::SX::sym("y");
    theta  = casadi::SX::sym("theta");
    states = casadi::SX::vertcat({x,y,theta});
    n_states = states.numel();

    v        = casadi::SX::sym("v");
    omega    = casadi::SX::sym("omega");
    controls = casadi::SX::vertcat({v,omega});
    n_controls = controls.numel();

    // 상태 공간 모델의 오른쪽 항을 정의함.
    // 상태 공간 모델에서 '오른쪽 항'은 시스템의 다이내믹을 나타내는 식에서 독립 변수인 시간에 대한 변화율을 말한다.
    rhs = casadi::SX::vertcat({v*casadi::SX::cos(theta),
                               v*casadi::SX::sin(theta),
                               omega});

    f = casadi::Function("f",{states,controls},{rhs});
    // 제어 변수
    U = casadi::SX::sym("U", n_controls, N);
    // 상태 변수
    X = casadi::SX::sym("X", n_states, N + 1);
    // 매개 변수 , 현재 상태 + 앞으로 N개의 노드에서의 상태 변수와 제어 변수 만큼 크기 선언
    P = casadi::SX::sym("P", n_states + N * (n_states + n_controls));

    // 비용 함수 가중치 (x, y, theta)
    Q = casadi::SX::diagcat({Q_x,Q_y,Q_theta});
    // 비용 함수 가중치 (R1 = 선속도, R2 = 각속도)
    R = casadi::SX::diagcat({R1, R2});
    // 제약 조건의 상한과 하한
    lbg = casadi::DM::zeros(n_states * (N + 1) , 1);
    ubg = casadi::DM::zeros(n_states * (N + 1) , 1);
    // 상태 변수, 제어 변수의 상한과 하한
    lbx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);
    ubx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);

    mpc_init_lbxubx();

    args.insert(std::pair<std::string,casadi::DM>("lbg",lbg));
    args.insert(std::pair<std::string,casadi::DM>("ubg",ubg));
    args.insert(std::pair<std::string,casadi::DM>("lbx",lbx));
    args.insert(std::pair<std::string,casadi::DM>("ubx",ubx));

    // vertcat 함수는 여러 요소를 수평으로 쌓는 함수임.
    rbt1_state_init = casadi::DM::vertcat({0.0, 0.0, 0.0});
    U0 = casadi::DM::zeros(N,n_controls);
    // rbt1_state_init 를 (1행)수평으로 N+1번 복사하여 하나의 큰 행렬로 만든다.
    X0 = casadi::DM::repmat(rbt1_state_init,1,N + 1);
}

void new_mpc3::mpc_init_lbxubx(){
        // x 위치, y 위치, theta 범위에 대한 제약 조건을 각 노드마다 설정
    for(int idx = 0; idx < n_states * (N + 1); ++idx)
    {
        if(idx % n_states == 0)
        {
            lbx(idx) = x_min;
            ubx(idx) = x_max;
        }
        else if(idx % n_states == 1)
        {
            lbx(idx) = y_min;
            ubx(idx) = y_max;
        }
        else if(idx % n_states == 2)
        {
            lbx(idx) = theta_min;
            ubx(idx) = theta_max;
        }
    }

    bool idx_even = true;

    //짝수 번째
    if((N + 1) % 2 == 0)
    {
        idx_even = true;
    }
    //홀수 번째
    else
    {
        idx_even = false;
    }
    // v, omega 에 대한 제약 조건 설정. ( 이거는 각 노드마다 아니고 마지막에 한 번만 )
    // 짝수와 홀수 구분하는 이유는 v가 먼저 들어가고 그 다음에 omega 가 들어가야됨.
    for(int idx = n_states * (N + 1); idx < lbx.size1(); ++idx)
    {
        // 짝수 번째
        if(idx_even)
        {
            // 짝수
            if(idx % n_controls == 0)
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
            // 홀수
            else
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
        }
        //홀수 번째
        else
        {
            // 짝수
            if(idx % n_controls == 0)
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
            // 홀수
            else
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
        }
    }
}

void new_mpc3::setting_solver(){
    casadi::SX obj = 0;
    casadi::SXVector g_vec ;
    // 예측한 현재 상태와 실제 현재 상태 비교
    for(int idx=0; idx < n_states; ++idx){
        g_vec.emplace_back(X(idx, 0) - P(idx));
    }
    g = casadi::SX::vertcat(g_vec);
    // 각 시간 스텝 별 예측한 상태들과 목표 상태(ref_path)들 비교
    casadi::SXVector st_vec;
    for(int n = 0; n < N+1; ++n){
        
    }
}

void new_mpc3::run()
{
    rbt1_odom_update();
    rbt1_ref_traj_update();
    if(rbt1_odom_in && rbt1_ref_traj_in)
    {
        state_update();
        make_args_p();
        setting_reference();
        reshape_and_init_opt_variable();
    }

    else
    {
        std::cout << "robot odom not subscribed..."<< std::endl;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<new_mpc3>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
