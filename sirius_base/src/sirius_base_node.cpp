#include "rclcpp/rclcpp.hpp"
#include "sirius_base/sirius_base.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto sirius_base = std::make_shared<sirius_base::SiriusBase>(rclcpp::NodeOptions());
    exec.add_node(sirius_base);
    exec.spin();
    rclcpp::shutdown();
}