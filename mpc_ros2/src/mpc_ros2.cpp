#include "mpc_ros2.hpp"
// double pi_to_pi(double angle)
// {
//     double result;
//     if(angle >= M_PI)
//     {
//         result = angle - 2.*M_PI;
//     }
//     if(angle <= -M_PI)
//     {
//         result = angle + 2.*M_PI;
//     }
//     return result;
// }



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
    // q.normalize();
    return q;
}

void mpc_ros2::rbt1_odom_Callbck(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    buf.lock();
    rbt1_odom_queue.push(*msg);
    buf.unlock();
}

void mpc_ros2::rbt1_ref_traj_Callbck(const nav_msgs::msg::Path::SharedPtr msg)
{
    buf.lock();
    rbt1_ref_traj_queue.push(*msg);
    buf.unlock();
}

void mpc_ros2::collision_avoidance()
{
    double rob_diam = 0.20;
    double safe_dis = 0.25;

    double curr_x = (double)rbt1_state_init(0);
    double curr_y = (double)rbt1_state_init(1);
    int n_avoid_obs = 0;

    for(int k = 0; k < N + 1; ++k)
    {
        for(int j = 0; j < n_obstacle; ++j)
        {
            double distance = sqrt(pow(curr_x - obstacle_vec[j][0],2) + pow(curr_y - obstacle_vec[j][1],2));
            if(distance > 0.8)
            {
                continue;
            }
            else
            {
                if(k == 0)
                {
                    ++n_avoid_obs;
                }
                g = casadi::SX::vertcat({g,-casadi::SX::sqrt(casadi::SX::pow(X(0,k) - obstacle_vec[j][0],2) + 
                    casadi::SX::pow(X(1,k) - obstacle_vec[j][1],2)) + (rob_diam + obstacle_vec[j][2])/2 + safe_dis});
            }   
        }
    }

    lbg = casadi::DM::zeros(n_states * (N + 1) + n_avoid_obs * (N + 1), 1);
    ubg = casadi::DM::zeros(n_states * (N + 1) + n_avoid_obs * (N + 1), 1);
    
    for(int idx = n_states * (N + 1); idx < lbg.size1(); ++ idx)
    {
        lbg(idx) = -casadi::inf;
    }
    args["lbg"] = lbg;
    args["ubg"] = ubg;
}

void mpc_ros2::reset()
{
    rbt1_pre_msg.poses.clear();
    rbt1_ref_msg.poses.clear();

    traj_make_time += 0.05;
    ++loop_count;

    rbt1_odom_in = false;
}

void mpc_ros2::publish()
{
    rbt1_pose_pub -> publish(rbt1_pose_msg);
    rbt1_cmd_pub -> publish(rbt1_cmd_msg);
    rbt1_pre_pub -> publish(rbt1_pre_msg);
}

void mpc_ros2::cmd_vel_msg_set()
{
    rbt1_cmd_msg.linear.x = (double)U0(0,0);
    rbt1_cmd_msg.angular.z = (double)U0(1,0);
}

void mpc_ros2::predictive_traj_msg_set()
{
    
    rbt1_pre_msg.header.frame_id = "odom";
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

void mpc_ros2::current_pos_msg_set()
{
    rbt1_pose_msg.header.frame_id = "odom";
    rbt1_pose_msg.header.stamp = ros2time.now();
    rbt1_pose_msg.point.x = (double)rbt1_state_init(0);
    rbt1_pose_msg.point.y = (double)rbt1_state_init(1);
}

void mpc_ros2::shift()
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

void mpc_ros2::get_result()
{
    casadi::Slice slice_x0(0,n_states*(N+1));
    casadi::DM X0_temp = solver_result["x"](slice_x0);
    X0 = casadi::DM::reshape(X0_temp,n_states,N+1);

    casadi::Slice slice_u0(n_states*(N+1),solver_result["x"].size1());
    casadi::DM U0_temp = solver_result["x"](slice_u0);
    U0 = casadi::DM::reshape(U0_temp,n_controls,N);

}

void mpc_ros2::call_solver()
{
    solver_result = solver(args);
}

void mpc_ros2::reshape_and_init_opt_variable()
{
    args.insert(std::pair<std::string,casadi::DM>("x0",casadi::DM::vertcat({casadi::DM::reshape(X0,n_states * (N+1),1) , 
                                                  casadi::DM::reshape(U0,n_controls * N,1)})));
    
}

void mpc_ros2::setting_reference()
{
    for(int n = 0; n < N; ++n)
    {
        double roll, pitch;
        double theta_ref = 0.0;
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

void mpc_ros2::make_args_p()
{
    casadi::DM p = casadi::DM::zeros((n_states + n_controls) * N + n_states,1);
    
    args.insert(std::pair<std::string,casadi::DM>("p",p));
    
    for(int idx = 0; idx < n_states; ++idx)
    {
        args["p"](idx) = rbt1_state_init(idx);
    }
}

void mpc_ros2::setting_solver()
{
    casadi::SX obj = 0;
    casadi::SXVector g_vec;
    for(int i = 0; i < n_states; ++i)
    {
        g_vec.emplace_back(X(i,0) - P(i));
    }
    g = casadi::SX::vertcat(g_vec);

    for(int n = 0; n < N; ++n)
    {
        casadi::SXVector st_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n));
        }
        casadi::SX st = casadi::SX::vertcat(st_vec);

        casadi::SXVector con_vec;
        for(int idx = 0; idx < n_controls; ++idx)
        {
            con_vec.emplace_back(U(idx,n));
        }
        casadi::SX con = casadi::SX::vertcat(con_vec);

        casadi::SXVector p_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            p_vec.emplace_back(st(idx) - P((n_states + n_controls) * n + 3 + idx,0));
        }
        casadi::SX state_err = casadi::SX::vertcat(p_vec);

        p_vec.clear();
        for(int idx = 0; idx < n_controls; ++idx)
        {
            p_vec.emplace_back(con(idx) - P((n_states + n_controls) * n + 6 + idx,0));
        }
        casadi::SX con_err = casadi::SX::vertcat(p_vec);

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
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n+1));
        }
        casadi::SX st_next = casadi::SX::vertcat(st_vec);
        casadi::SXVector f_value_vec = f(casadi::SXVector{st,con});
        casadi::SX f_value = casadi::SX::vertcat(f_value_vec);
        casadi::SX st_next_euler = st + (dt * f_value);
        g = casadi::SX::vertcat({g,st_next - st_next_euler});
    }

    if(n_obstacle != 0)
    {
        collision_avoidance();
    }
    
    casadi::SX OPT_variables = casadi::SX::vertcat({casadi::SX::reshape(X,-1,1),casadi::SX::reshape(U,-1,1)});
    casadi::SXDict nlp_prob;

    nlp_prob.insert(std::pair<std::string,casadi::SX>("f",obj));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("x",OPT_variables));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("g",g));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("p",P));
    // std::cout << nlp_prob["g"].size() << std::endl;
    casadi::Dict opts = {
    {"ipopt", casadi::Dict{
        {"max_iter", 200},
        {"print_level", 0},
        {"acceptable_tol", 1e-8},
        {"acceptable_obj_change_tol", 1e-6}
    }},
    {"print_time", 0}
    };
    solver = casadi::nlpsol("solver","ipopt",nlp_prob,opts);
}

void mpc_ros2::state_update()
{
    double rbt1_yaw = pi_to_pi(q_to_e(rbt1_odom_msg));
    rbt1_state_init = casadi::DM::vertcat({rbt1_odom_msg.pose.pose.position.x, rbt1_odom_msg.pose.pose.position.y, rbt1_yaw});
    // rbt1_state_init = casadi::DM::vertcat({1.0, 0.5, 0.002});
    std::cout << "rbt1_yaw : " << rbt1_yaw << std::endl;
}

void mpc_ros2::rbt1_ref_traj_update()
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

void mpc_ros2::rbt1_odom_update()
{
    if(!rbt1_odom_queue.empty())
    {
        buf.lock();
        rbt1_odom_msg = rbt1_odom_queue.back();
        rbt1_odom_queue = std::queue<nav_msgs::msg::Odometry>();
        rbt1_odom_in = true;
        buf.unlock();
    }
}

void mpc_ros2::obstacle_set()
{
    std::vector<double> obstacle1 = {4.0, 0.3, 1.0};
    std::vector<double> obstacle2 = {7.5, -0.6, 1.0};
    std::vector<double> obstacle3 = {11.0, 0.5, 1.0};
    std::vector<double> obstacle4 = {16.0, -0.4, 1.0};
    obstacle_vec.emplace_back(obstacle1);
    obstacle_vec.emplace_back(obstacle2);
    obstacle_vec.emplace_back(obstacle3);
    obstacle_vec.emplace_back(obstacle4);
    obstacle_vec.clear();
    n_obstacle = obstacle_vec.size();
}

void mpc_ros2::mpc_init()
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

    rhs = casadi::SX::vertcat({v*casadi::SX::cos(theta),
                               v*casadi::SX::sin(theta),
                               omega});

    f = casadi::Function("f",{states,controls},{rhs});
    U = casadi::SX::sym("U", n_controls, N);
    X = casadi::SX::sym("X", n_states, N + 1);
    P = casadi::SX::sym("P", n_states + N * (n_states + n_controls));

    Q = casadi::SX::diagcat({Q_x,Q_y,Q_theta});
    R = casadi::SX::diagcat({R1, R2});


   

    lbg = casadi::DM::zeros(n_states * (N + 1) + n_obstacle * (N + 1), 1);
    ubg = casadi::DM::zeros(n_states * (N + 1) + n_obstacle * (N + 1), 1);
    
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

    rbt1_state_init = casadi::DM::vertcat({0.0,0.0,0.0});
    U0 = casadi::DM::zeros(N,n_controls);
    X0 = casadi::DM::repmat(rbt1_state_init,1,N + 1);
}

void mpc_ros2::init()
{
    rbt1_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/robot1/odom",1,std::bind(&mpc_ros2::rbt1_odom_Callbck,this,_1));
    rbt1_ref_traj_sub = this->create_subscription<nav_msgs::msg::Path>("/robot1/ref_trajectory",1,std::bind(&mpc_ros2::rbt1_ref_traj_Callbck,this,_1));

    rbt1_pose_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/robot1/pose",1);
    rbt1_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel",1);
    // rbt1_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/rbt1/cmd_vel",1);
    rbt1_pre_pub = this->create_publisher<nav_msgs::msg::Path>("/robot1/pre_path",1);
}

void mpc_ros2::run()
{
    double time;
    TicToc T;
    obstacle_set();
    mpc_init();
    rclcpp::WallRate r(10);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());
        // rclcpp::executor::Executor::spin_once;
        rbt1_odom_update();
        rbt1_ref_traj_update();
        if(rbt1_odom_in && rbt1_ref_traj_in)
        {   
            T.tic();
            state_update();
            setting_solver();
            make_args_p();
            // desired_trajectory();
            setting_reference();
            reshape_and_init_opt_variable();
            call_solver();
            get_result();
            time = T.toc();
            current_pos_msg_set();
            cmd_vel_msg_set();
            predictive_traj_msg_set();
            publish();
            shift();
            reset();
            casadi::Dict solver_iter = solver.stats();
            std::cout << "=================================="<< std::endl;
            std::cout << "loop time        : " << time << std::endl;
            std::cout << "loop num         : " << loop_count << std::endl;
            std::cout << "solver iteration : " << solver_iter["iter_count"]<< std::endl;
            std::cout << "solver cost      : " << solver_result["f"] << std::endl;
            std::cout << "robot vel      : " << rbt1_cmd_msg.linear.x << std::endl;
        }

        else
        {
            std::cout << "robot odom && trajectory not subscribed..."<< std::endl;
        }

        r.sleep();
    }
}

