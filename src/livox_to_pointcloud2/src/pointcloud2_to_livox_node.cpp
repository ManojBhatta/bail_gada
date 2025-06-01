// pointcloud2_to_livox_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "pointcloud2_to_livox.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud2ToLivox>());
    rclcpp::shutdown();
    return 0;
}
