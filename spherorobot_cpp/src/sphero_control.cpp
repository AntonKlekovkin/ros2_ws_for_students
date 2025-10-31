#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("platform_control_node");
    
    while(rclcpp::ok())
    {
        double currentTime = node->now().seconds();
        
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    rclcpp::shutdown();
    return 0;
}
