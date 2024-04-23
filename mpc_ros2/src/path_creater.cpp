#include "path_creater.hpp"

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
    angle -= 2.*M_PI;

  while(angle < -M_PI)
    angle += 2.*M_PI;

  return angle;
}

tf2::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    // q.normalize();
    return q;
}

void path_creater::reset()
{
    lead_ref_traj_msg.poses.clear();
    f1_ref_traj_msg.poses.clear();
    f2_ref_traj_msg.poses.clear();
    f3_ref_traj_msg.poses.clear();
    f4_ref_traj_msg.poses.clear();

    ++make_ref_id;
}

void path_creater::publish()
{
    global_path_msg.header.frame_id = "odom";
    lead_ref_traj_msg.header.frame_id = "odom";
    f1_ref_traj_msg.header.frame_id = "odom";
    f2_ref_traj_msg.header.frame_id = "odom";
    f3_ref_traj_msg.header.frame_id = "odom";
    f4_ref_traj_msg.header.frame_id = "odom";

    global_path_msg.header.stamp = ros2time.now();
    lead_ref_traj_msg.header.stamp = ros2time.now();
    f1_ref_traj_msg.header.stamp = ros2time.now();
    f2_ref_traj_msg.header.stamp = ros2time.now();
    f3_ref_traj_msg.header.stamp = ros2time.now();
    f4_ref_traj_msg.header.stamp = ros2time.now();

    global_path_pub -> publish(global_path_msg);
    lead_ref_traj_pub -> publish(lead_ref_traj_msg);
    f1_ref_traj_pub -> publish(f1_ref_traj_msg);
    f2_ref_traj_pub -> publish(f2_ref_traj_msg);
    f3_ref_traj_pub -> publish(f3_ref_traj_msg);
    f4_ref_traj_pub -> publish(f4_ref_traj_msg);
}

void path_creater::make_follow_ref_trajectory()
{
    for(const auto& lead_node : lead_ref_traj_msg.poses)
    {
        geometry_msgs::msg::PoseStamped f1_node = lead_node;
        geometry_msgs::msg::PoseStamped f2_node = lead_node;
        geometry_msgs::msg::PoseStamped f3_node = lead_node;
        geometry_msgs::msg::PoseStamped f4_node = lead_node;

        f1_node.pose.position.x += f1_x;
        f1_node.pose.position.y += f1_y;
        f2_node.pose.position.x += f2_x;
        f2_node.pose.position.y += f2_y;
        f3_node.pose.position.x += f3_x;
        f3_node.pose.position.y += f3_y;
        f4_node.pose.position.x += f4_x;
        f4_node.pose.position.y += f4_y;

        f1_ref_traj_msg.poses.emplace_back(f1_node);
        f2_ref_traj_msg.poses.emplace_back(f2_node);
        f3_ref_traj_msg.poses.emplace_back(f3_node);
        f4_ref_traj_msg.poses.emplace_back(f4_node);
    }
}

void path_creater::make_lead_ref_theta()
{
    geometry_msgs::msg::PoseStamped ref_point_st;
    geometry_msgs::msg::PoseStamped ref_point_end;
    if(!traj_end)
    {
        for(int i = 0; i < node ; ++i)
        {
            double ref_angle;
            tf2::Quaternion q;

            ref_point_st = lead_ref_traj_msg.poses[i];
            ref_point_end = lead_ref_traj_msg.poses[i+1];

            ref_angle = atan2(ref_point_end.pose.position.y - ref_point_st.pose.position.y,
                              ref_point_end.pose.position.x - ref_point_st.pose.position.x);

            q = e_to_q(0.0,0.0,pi_to_pi(ref_angle));
            
            if(i < node-1)
            {
                lead_ref_traj_msg.poses[i].pose.orientation.x = q.x();
                lead_ref_traj_msg.poses[i].pose.orientation.y = q.y();
                lead_ref_traj_msg.poses[i].pose.orientation.z = q.z();
                lead_ref_traj_msg.poses[i].pose.orientation.w = q.w();
            }
            else
            {
                lead_ref_traj_msg.poses[i].pose.orientation.x = lead_ref_traj_msg.poses[i-1].pose.orientation.x;
                lead_ref_traj_msg.poses[i].pose.orientation.y = lead_ref_traj_msg.poses[i-1].pose.orientation.y;
                lead_ref_traj_msg.poses[i].pose.orientation.z = lead_ref_traj_msg.poses[i-1].pose.orientation.z;
                lead_ref_traj_msg.poses[i].pose.orientation.w = lead_ref_traj_msg.poses[i-1].pose.orientation.w;
            }

            if(lead_ref_traj_msg.poses[node-1].pose.position.x == global_path_msg.poses[global_path_msg.poses.size()-1].pose.position.x && lead_ref_traj_msg.poses[node-1].pose.position.y == global_path_msg.poses[global_path_msg.poses.size()-1].pose.position.y)
            {
                traj_end = true;
                last_q.setX(lead_ref_traj_msg.poses[node-2].pose.orientation.x);
                last_q.setY(lead_ref_traj_msg.poses[node-2].pose.orientation.y);
                last_q.setZ(lead_ref_traj_msg.poses[node-2].pose.orientation.z);
                last_q.setW(lead_ref_traj_msg.poses[node-2].pose.orientation.w);
            }
        }
    }
    else
    {
        for(int i = 0; i < node; ++i)
        {
            lead_ref_traj_msg.poses[i].pose.orientation.x = last_q.x();
            lead_ref_traj_msg.poses[i].pose.orientation.y = last_q.y();
            lead_ref_traj_msg.poses[i].pose.orientation.z = last_q.z();
            lead_ref_traj_msg.poses[i].pose.orientation.w = last_q.w();
        }
    }
}

void path_creater::make_lead_ref_trajectory()
{
    for(int i = make_ref_id ; i < make_ref_id + node; ++i)
    {
        if(i <= global_node_max - 1)
        {   
            lead_ref_traj_msg.poses.emplace_back(global_path_msg.poses[i]);
        }
        else
        {
            lead_ref_traj_msg.poses.emplace_back(global_path_msg.poses[global_node_max-1]);
            std::cout << global_path_msg.poses[global_node_max-1].pose.position.x;
        }
    }
    
}

void path_creater::global_path()
{
    geometry_msgs::msg::PoseStamped global_node;
    double global_x = 0;
    global_node_max = x_max/(v_mean * (1.0/rate));
    std::cout << "MODE : " << MODE << " straight path create" << std::endl;
    std::cout << "global node max : "<< global_node_max << std::endl;
    for(int i = 0; i < global_node_max; ++i)
    {
        global_node.pose.position.x = global_x + (x_max / global_node_max) * i;
        if(MODE == 0)
        {
            global_node.pose.position.y = 0;
        }
        else if(MODE == 1)
        {  
            global_node.pose.position.y = sin(global_node.pose.position.x/10 * 4 * M_PI) * curve_y_max;
        }
        global_path_msg.poses.emplace_back(global_node);
    }
}

void path_creater::init()
{
    global_path_pub = this -> create_publisher<nav_msgs::msg::Path>("/global_path",1);
    lead_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/robot1/ref_trajectory",1);
    f1_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/robot2/ref_trajectory",1);
    f2_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/robot3/ref_trajectory",1);
    f3_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/robot4/ref_trajectory",1);
    f4_ref_traj_pub = this -> create_publisher<nav_msgs::msg::Path>("/robot5/ref_trajectory",1);
}

void path_creater::run()
{
    global_path();
    rclcpp::WallRate r(rate);
    while(rclcpp::ok)
    {
        rclcpp::spin_some(shared_from_this());
        make_lead_ref_trajectory();
        make_lead_ref_theta();
        make_follow_ref_trajectory();
        publish();
        reset();
        r.sleep();
    }
}
