#include "global_path_creater.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<global_path_creater>();
    node -> init();
    node -> run();
    
    return 0;
}