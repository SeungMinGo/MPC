#include "global_path_creater.hpp"

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

void global_path_creater::global_path_create()
{
    geometry_msgs::msg::PoseStamped global_node;
    double global_x = 0;
    for(int i = 0; i < global_node_max; ++i)
    {
        global_node.pose.position.x = global_x + (x_max / global_node_max) * i;
        global_node.pose.position.y = sin(global_node.pose.position.x / 10 * 2 * M_PI);

        rbt1_global_path_msg.poses.emplace_back(global_node);
    }
    global_x_max = rbt1_global_path_msg.poses[rbt1_global_path_msg.poses.size()-1].pose.position.x;
}

void global_path_creater::rbt1_odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    buf.lock();
    rbt1_odom_queue.push(*msg);
    buf.unlock();
}

void global_path_creater::reset()
{
    rbt1_odom_in = false;
    rbt1_ref_traj_msg.poses.clear();
    traj_make_time += 0.1;
    std::cout << rbt1_global_path_msg.poses[rbt1_global_path_msg.poses.size()-1].pose.position.y << std::endl;
}

void global_path_creater::publish()
{
    rbt1_global_path_pub -> publish(rbt1_global_path_msg);
    rbt1_ref_traj_pub -> publish(rbt1_ref_traj_msg);
}

void global_path_creater::ref_traj_msg_set()
{
    rbt1_ref_traj_msg.header.frame_id = "odom";
    rbt1_ref_traj_msg.header.stamp = ros2time.now();
}

void global_path_creater::global_path_msg_set()
{
    rbt1_global_path_msg.header.frame_id = "odom";
    rbt1_global_path_msg.header.stamp = ros2time.now();
}

void global_path_creater::ref_traj_angle_set()
{
    geometry_msgs::msg::PoseStamped ref_point_st;
    geometry_msgs::msg::PoseStamped ref_point_end;
    if(!traj_end)
    {
        for(int i = 0; i < N; ++i)
        {
            double ref_angle;
            tf2::Quaternion q;
            if(i < N-1)
            {
                ref_point_st = rbt1_ref_traj_msg.poses[i];
                ref_point_end = rbt1_ref_traj_msg.poses[i+1];

                ref_angle = atan2(ref_point_end.pose.position.y - ref_point_st.pose.position.y,
                                  ref_point_end.pose.position.x - ref_point_st.pose.position.x);

                q = e_to_q(0.0,0.0,pi_to_pi(ref_angle));

                rbt1_ref_traj_msg.poses[i].pose.orientation.x = q.x();
                rbt1_ref_traj_msg.poses[i].pose.orientation.y = q.y();
                rbt1_ref_traj_msg.poses[i].pose.orientation.z = q.z();
                rbt1_ref_traj_msg.poses[i].pose.orientation.w = q.w();

                last_q = q;
            }
            else
            {
                rbt1_ref_traj_msg.poses[i].pose.orientation.x = rbt1_ref_traj_msg.poses[i-1].pose.orientation.x;
                rbt1_ref_traj_msg.poses[i].pose.orientation.y = rbt1_ref_traj_msg.poses[i-1].pose.orientation.y;
                rbt1_ref_traj_msg.poses[i].pose.orientation.z = rbt1_ref_traj_msg.poses[i-1].pose.orientation.z;
                rbt1_ref_traj_msg.poses[i].pose.orientation.w = rbt1_ref_traj_msg.poses[i-1].pose.orientation.w;
            }

            if(rbt1_ref_traj_msg.poses[i].pose.position.x == global_x_max)
            {
                traj_end = true;
                rbt1_ref_traj_msg.poses[i].pose.orientation.x = rbt1_ref_traj_msg.poses[i-1].pose.orientation.x;
                rbt1_ref_traj_msg.poses[i].pose.orientation.y = rbt1_ref_traj_msg.poses[i-1].pose.orientation.y;
                rbt1_ref_traj_msg.poses[i].pose.orientation.z = rbt1_ref_traj_msg.poses[i-1].pose.orientation.z;
                rbt1_ref_traj_msg.poses[i].pose.orientation.w = rbt1_ref_traj_msg.poses[i-1].pose.orientation.w;
            }
            else
            {
                traj_end = false;
            }
        }
    }
    
}

void global_path_creater::ref_traj_create()
{
    geometry_msgs::msg::PoseStamped ref_node;
    double ref_x = ref_traj_start;
    for(int i = 0; i < N; ++i)
    { //dt-> ref_path 안에 쪼개진 노드 사이의 시간, N-> 노드 개수
        ref_node.pose.position.x = (traj_make_time + i *dt) / (dt * N);
        ref_node.pose.position.y = sin(ref_node.pose.position.x / 10 * 2 * M_PI);
        if(i == 1)
        {
            ref_traj_start = ref_node.pose.position.x;
        }
        if(ref_node.pose.position.x > global_x_max)
        {
            ref_node.pose.position.x = global_x_max;
            ref_node.pose.position.y = rbt1_global_path_msg.poses[rbt1_global_path_msg.poses.size()-1].pose.position.y;
        }
        rbt1_ref_traj_msg.poses.emplace_back(ref_node);
    }
    if(traj_end)
    {
        for(int i = 0; i < N; ++i)
        {
            rbt1_ref_traj_msg.poses[i].pose.orientation.x = last_q.x();
            rbt1_ref_traj_msg.poses[i].pose.orientation.y = last_q.y();
            rbt1_ref_traj_msg.poses[i].pose.orientation.z = last_q.z();
            rbt1_ref_traj_msg.poses[i].pose.orientation.w = last_q.w();
        }
    }
}

void global_path_creater::rbt1_odom_update()
{
    if(!rbt1_odom_queue.empty())
    {
        buf.lock();
        rbt1_odom_queue.pop();
        rbt1_odom_in = true;
        buf.unlock();
    }
}   

void global_path_creater::init()
{
    rbt1_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",1,std::bind(&global_path_creater::rbt1_odom_Callback,this,_1));
    rbt1_global_path_pub = this -> create_publisher<nav_msgs::msg::Path>("/rbt1/global_path",1); 
    rbt1_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/rbt1/ref_trajectory",1); 
}

void global_path_creater::run()
{
    global_path_create();
    rclcpp::WallRate r(10);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());
        // rclcpp::executor::Executor::spin_once;
        rbt1_odom_update();
        if(rbt1_odom_in)
        {   
            ref_traj_create();
            ref_traj_angle_set();
            ref_traj_msg_set();
            global_path_msg_set();
            publish();
            reset();
            std::cout << "robot global path pub ..."<< std::endl;
        }
        else
        {
            std::cout << "robot odom not subscribed..."<< std::endl;
        }
        r.sleep();
    }
}