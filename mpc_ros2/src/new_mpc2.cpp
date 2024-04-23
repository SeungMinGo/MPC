#include "new_mpc2.hpp"

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

tf2::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

void new_mpc2::init()
{
    rbt1_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",1,std::bind(&new_mpc2::rbt1_odom_Callbck,this,_1));
    rbt1_ref_traj_sub = this->create_subscription<nav_msgs::msg::Path>("/leader/ref_trajectory",1,std::bind(&new_mpc2::rbt1_ref_traj_Callbck,this,_1));
    rbt1_pose_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/leader/pose",1);
    rbt1_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    rbt1_pre_pub = this->create_publisher<nav_msgs::msg::Path>("/leader/pre_path",1);
}

void new_mpc2::rbt1_odom_Callbck(const nav_msgs::msg::Odometry::SharedPtr msg){
    buf.lock();
    rbt1_odom_queue.push(*msg);
    buf.unlock();
}

void new_mpc2::rbt1_ref_traj_Callbck(const nav_msgs::msg::Path::SharedPtr msg){
    buf.lock();
    rbt1_ref_traj_queue.push(*msg);
    buf.unlock();
}

void new_mpc2::rbt1_odom_update()
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

void new_mpc2::state_update()
{
    double rbt1_yaw = pi_to_pi(q_to_e(rbt1_odom_msg));
    rbt1_state_init = casadi::DM::vertcat({rbt1_odom_msg.pose.pose.position.x, rbt1_odom_msg.pose.pose.position.y, rbt1_yaw});
    // rbt1_state_init = casadi::DM::vertcat({1.0, 0.5, 0.002});
}

void new_mpc2::rbt1_ref_traj_update()
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

void new_mpc2::make_args_p()
{
    casadi::DM p = casadi::DM::zeros((n_states + n_controls) * N + n_states,1);

    args.insert(std::pair<std::string,casadi::DM>("p",p));

    for(int idx = 0; idx < n_states; ++idx)
    {
        args["p"](idx) = rbt1_state_init(idx);
    }
}

void new_mpc2::setting_reference()
{
    for(int n = 0; n < N; ++n)
    {
        double roll, pitch;
        double theta_ref = 0.0;
        // 이 속도의 역할 의문
        double u_ref = 0.5 / 6.0;
        double omega_ref = 0.0;

        tf2::Quaternion q;
        q.setX(rbt1_ref_traj_msg.poses[n].pose.orientation.x);
        q.setY(rbt1_ref_traj_msg.poses[n].pose.orientation.y);
        q.setZ(rbt1_ref_traj_msg.poses[n].pose.orientation.z);
        q.setW(rbt1_ref_traj_msg.poses[n].pose.orientation.w);

        tf2::Matrix3x3 m(q);
        m.getRPY(roll,pitch,theta_ref);

        args["p"]((n_states + n_controls) * n + 3) = rbt1_ref_traj_msg.poses[n].pose.position.x;
        args["p"]((n_states + n_controls) * n + 4) = rbt1_ref_traj_msg.poses[n].pose.position.y;
        args["p"]((n_states + n_controls) * n + 5) = theta_ref;
        args["p"]((n_states + n_controls) * n + 6) = u_ref;
        args["p"]((n_states + n_controls) * n + 7) = omega_ref;
    }
}

void new_mpc2::reshape_and_init_opt_variable()
{
            // 출력 코드 추가
    //std::cout << "X02 (State Trajectory):\n" << X0 << std::endl;
    //std::cout << "U02 (Control Trajectory):\n" << U0 << std::endl;
    //std::cout << "===========================" << std::endl;
    args.insert(std::pair<std::string,casadi::DM>("x0",casadi::DM::vertcat({casadi::DM::reshape(X0,n_states * (N+1),1) ,
                                                  casadi::DM::reshape(U0,n_controls * N,1)})));
}

// 얘는 맨 처음 한 번만 실행됨.
void new_mpc2::mpc_init()
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
    U = casadi::SX::sym("U", n_controls, N);
    X = casadi::SX::sym("X", n_states, N + 1);
    // 현재 상태 + 앞으로 N개의 노드에서의 상태 변수와 제어 변수 만큼 크기 선언
    P = casadi::SX::sym("P", n_states + N * (n_states + n_controls));

    // 비용 함수 가중치 (x, y, theta)
    Q = casadi::SX::diagcat({Q_x,Q_y,Q_theta});
    // 비용 함수 가중치 (R1 = 선속도, R2 = 각속도)
    R = casadi::SX::diagcat({R1, R2});
        // termior

    lbg = casadi::DM::zeros(n_states * (N + 1) , 1);
    ubg = casadi::DM::zeros(n_states * (N + 1) , 1);

    lbx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);
    ubx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);

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

    if((N + 1) % 2 == 0)
    {
        idx_even = true;
    }
    else
    {
        idx_even = false;
    }

    for(int idx = n_states * (N + 1); idx < lbx.size1(); ++idx)
    {
        if(idx_even)
        {
            if(idx % n_controls == 0)
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
            else
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
        }
        else
        {
            if(idx % n_controls == 0)
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
            else
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
        }
    }

    args.insert(std::pair<std::string,casadi::DM>("lbg",lbg));
    args.insert(std::pair<std::string,casadi::DM>("ubg",ubg));
    args.insert(std::pair<std::string,casadi::DM>("lbx",lbx));
    args.insert(std::pair<std::string,casadi::DM>("ubx",ubx));

    // vertcat 함수는 여러 요소를 수평으로 쌓는 함수임. 
    rbt1_state_init = casadi::DM::vertcat({0.0, 0.0, 0.0});
    U0 = casadi::DM::zeros(N,n_controls);
    // rbt1_state_init 를 수평으로 N+1번 복사하여 하나의 큰 행렬로 만든다.
    X0 = casadi::DM::repmat(rbt1_state_init,1,N + 1);
}

void new_mpc2::setting_solver()
{
    // 초기 목적 함수를 0으로 설정
    casadi::SX obj = 0;
    // 제약 조건 벡터 초기화
    casadi::SXVector g_vec;
// 초기 상태 변수와 주어진 매개 변수 간의 오차
    // 초기 상태 변수와 매개변수 간의 오차 벡터를 계산하여 제약 조건 벡터에 추가
    for(int i = 0; i < n_states; ++i)
    {
        g_vec.emplace_back(X(i,0) - P(i));
    }
    // 제약 조건 벡터를 수평으로 쌓아 하나의 벡터로 합침
    g = casadi::SX::vertcat(g_vec);
// 각 시간 단계에서의 상태 변수 및 제어 변수와 매개 변수 간의 오차
    // 각 시간 스텝에서 상태 변수와 제어 변수를 사용하여 최적화 목적 함수 계산
    for(int n = 0; n < N; ++n)
    {
        // 현재 시간 스텝의 상태 변수 벡터 생성
        // 이거 st에 X 상태 변수 N개가 아니라 N+1개를 넣어주는 수정이 필요할 것 같음.
        casadi::SXVector st_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n));
        }
        casadi::SX st = casadi::SX::vertcat(st_vec);

        // 현재 시간 스텝의 제어 변수 벡터 생성
        casadi::SXVector con_vec;
        for(int idx = 0; idx < n_controls; ++idx)
        {
            con_vec.emplace_back(U(idx,n));
        }
        casadi::SX con = casadi::SX::vertcat(con_vec);

        // 상태 변수와 매개 변수간의 오차를 계산
        casadi::SXVector p_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            p_vec.emplace_back(st(idx) - P((n_states + n_controls) * n + 3 + idx,0));
        }
        casadi::SX state_err = casadi::SX::vertcat(p_vec);

        // p_vec 초기화
        p_vec.clear();
        // 제어 변수와 매개 변수간의 오차를 계산
        for(int idx = 0; idx < n_controls; ++idx)
        {
            p_vec.emplace_back(con(idx) - P((n_states + n_controls) * n + 6 + idx,0));
        }
        casadi::SX con_err = casadi::SX::vertcat(p_vec);

        // 상태 변수와 제어 변수의 오차 벡터 계산 및 비용 함수에 추가
        if(n >= N - 1)
        {
            obj = obj + gamma * casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err);
        }
        else
        {
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err) + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err);
        }

        // obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err) + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err);


        st_vec.clear();
        // 현재 시간 말고 다음 시간에서의 상태 변수를 'st_vec'에 추가함
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n+1));
        }
        casadi::SX st_next = casadi::SX::vertcat(st_vec);
        casadi::SXVector f_value_vec = f(casadi::SXVector{st,con});
        casadi::SX f_value = casadi::SX::vertcat(f_value_vec);
        // 다음 시간 스텝의 상태 변수 예측값 계산
        casadi::SX st_next_euler = st + (dt * f_value);
        // 상태 변수 예측값과 다음 시간 스텝의 상태 변수 간의 오차를 제약 조건 벡터에 추가
        g = casadi::SX::vertcat({g,st_next - st_next_euler});
    }

    // 최적화 변수 및 제약 조건을 포함한 최적화 문제의 변수 생성
    casadi::SX OPT_variables = casadi::SX::vertcat({casadi::SX::reshape(X,-1,1),casadi::SX::reshape(U,-1,1)});
    // 최적화 문제 변수, 목적 함수, 제약 조건을 포함하는 사전 생성
    casadi::SXDict nlp_prob;
    nlp_prob.insert(std::pair<std::string,casadi::SX>("f",obj));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("x",OPT_variables));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("g",g));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("p",P));
    // 최적화 알고리즘 옵션 설정
    casadi::Dict opts = {
    {"ipopt", casadi::Dict{
        {"max_iter", 200},
        {"print_level", 0},
        {"acceptable_tol", 1e-8},
        {"acceptable_obj_change_tol", 1e-6}
    }},
    {"print_time", 0}
    };
    // 최적화 문제 생성 및 설정
    solver = casadi::nlpsol("solver","ipopt",nlp_prob,opts);
}

void new_mpc2::call_solver()
{
    solver_result = solver(args);
}

void new_mpc2::get_result()
{
    casadi::Slice slice_x0(0,n_states*(N+1));
    casadi::DM X0_temp = solver_result["x"](slice_x0);
    X0 = casadi::DM::reshape(X0_temp,n_states,N+1);

    casadi::Slice slice_u0(n_states*(N+1),solver_result["x"].size1());
    casadi::DM U0_temp = solver_result["x"](slice_u0);
    U0 = casadi::DM::reshape(U0_temp,n_controls,N);

        // 출력 코드 추가
    //std::cout << "X01 (State Trajectory):\n" << X0 << std::endl;
    //std::cout << "U01 (Control Trajectory):\n" << U0 << std::endl;
    //std::cout << "===========================" << std::endl;
}

void new_mpc2::cmd_vel_msg_set()
{
    rbt1_cmd_msg.linear.x = (double)U0(0,0);
    rbt1_cmd_msg.angular.z = (double)U0(1,0);
}

void new_mpc2::predictive_traj_msg_set()
{
    rbt1_pre_msg.header.frame_id = "world";
    rbt1_pre_msg.header.stamp = ros2time.now();
    for(int n = 0; n < N; ++n)
    {
        geometry_msgs::msg::PoseStamped n_path;
        n_path.pose.position.x = (double)X0(0,n);
        n_path.pose.position.y = (double)X0(1,n);

        tf2::Quaternion n_q = e_to_q(0.0,0.0,pi_to_pi((double)X0(2,n)));

        n_path.pose.orientation.x = n_q.x();
        n_path.pose.orientation.y = n_q.y();
        n_path.pose.orientation.z = n_q.z();
        n_path.pose.orientation.w = n_q.w();

        rbt1_pre_msg.poses.emplace_back(n_path);
    }
}

void new_mpc2::publish()
{
    rbt1_pose_pub -> publish(rbt1_pose_msg);
    rbt1_cmd_pub -> publish(rbt1_cmd_msg);
    rbt1_pre_pub -> publish(rbt1_pre_msg);
}

void new_mpc2::shift()
{
    casadi::DM X0_last = casadi::DM::vertcat({X0(0,-1),X0(1,-1),X0(2,-1)});
    X0 = casadi::DM::reshape(X0,X0.size1()*X0.size2(),1);
    X0 = casadi::DM::vertcat({X0(casadi::Slice(n_states,X0.size1()*X0.size2())),X0_last});
    X0 = casadi::DM::reshape(X0,n_states,N+1);

    casadi::DM U0_last = casadi::DM::vertcat({U0(0,-1),U0(1,-1)});
    U0 = casadi::DM::reshape(U0,U0.size1()*U0.size2(),1);
    U0 = casadi::DM::vertcat({U0(casadi::Slice(n_controls,U0.size1()*U0.size2())),U0_last});

    U0 = casadi::DM::reshape(U0,n_controls,N);
}

void new_mpc2::reset()
{
    ref_path.clear();
    rbt1_pre_msg.poses.clear();
    rbt1_ref_msg.poses.clear();

    ++loop_count;

    rbt1_odom_in = false;
}

void new_mpc2::run()
{
    double time;
    TicToc T;

    mpc_init();
    rclcpp::WallRate r(10);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());
        rbt1_odom_update();
        rbt1_ref_traj_update();
        if(rbt1_odom_in && rbt1_ref_traj_in)
        {
            T.tic();
            state_update();
            make_args_p();
            setting_reference();
            reshape_and_init_opt_variable();
            setting_solver();
            call_solver();
            get_result();
            time = T.toc();
            cmd_vel_msg_set();
            predictive_traj_msg_set();
            publish();
            shift();
            reset();
            /*
            casadi::Dict solver_iter = solver.stats();
            std::cout << "=================================="<< std::endl;
            std::cout << "loop time        : " << time << std::endl;
            std::cout << "loop num         : " << loop_count << std::endl;
            std::cout << "solver iteration : " << solver_iter["iter_count"]<< std::endl;
            std::cout << "solver cost      : " << solver_result["f"] << std::endl;*/
        }

        else
        {
            std::cout << "robot odom not subscribed..."<< std::endl;
        }

        r.sleep();
    }
}
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<new_mpc2>();
    node -> init();
    node -> run();
    return 0;
}