// pointcloud2_to_livox.cpp
#include "pointcloud2_to_livox.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

PointCloud2ToLivox::PointCloud2ToLivox() : Node("pointcloud2_to_livox")
{
    publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox_custom_msg", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pointcloud2", 10, std::bind(&PointCloud2ToLivox::callback, this, std::placeholders::_1));
}

void PointCloud2ToLivox::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    livox_ros_driver2::msg::CustomMsg output;
    output.header = msg->header;
    output.timebase = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    output.point_num = msg->width * msg->height;
    output.lidar_id = 0;  // Set appropriate lidar_id if known
    
    // Initialize mandatory field iterators
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    // Check for optional fields
    bool has_intensity = false;
    bool has_tag = false;
    bool has_line = false;
    
    std::optional<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity;
    std::optional<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_tag;
    std::optional<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_line;
    
    // Check which optional fields exist
    for (const auto& field : msg->fields) {
        if (field.name == "intensity") {
            has_intensity = true;
            iter_intensity.emplace(*msg, "intensity");
        }
        if (field.name == "tag") {
            has_tag = true;
            iter_tag.emplace(*msg, "tag");
        }
        if (field.name == "line") {
            has_line = true;
            iter_line.emplace(*msg, "line");
        }
    }

    // Reserve space for points
    output.points.reserve(output.point_num);

    // Convert points
    for (size_t i = 0; i < output.point_num; ++i, ++iter_x, ++iter_y, ++iter_z) {
        livox_ros_driver2::msg::CustomPoint point;
        
        // Set mandatory fields
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        
        // Set optional fields
        if (has_intensity) {
            point.reflectivity = static_cast<uint8_t>(**iter_intensity);
            ++(*iter_intensity);
        } else {
            point.reflectivity = 0;
        }
        
        if (has_tag) {
            point.tag = **iter_tag;
            ++(*iter_tag);
        } else {
            point.tag = 0;
        }
        
        if (has_line) {
            point.line = **iter_line;
            ++(*iter_line);
        } else {
            point.line = 0;
        }
        
        // Set offset time based on point index
        point.offset_time = static_cast<uint32_t>(i);
        
        output.points.push_back(point);
    }

    publisher_->publish(output);
}