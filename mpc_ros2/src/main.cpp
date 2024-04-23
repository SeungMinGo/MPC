#include "mpc_ros2.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<mpc_ros2>();
    node -> init();
    node -> run();
    
    return 0;
}