#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "csignal"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("platform_control_node");
 
    
    while(rclcpp::ok())
    {        
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    rclcpp::shutdown();
    return 0;
}
