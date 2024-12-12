#ifndef DENSE_DEPTH_FUSION_HPP
#define DENSE_DEPTH_FUSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"


namespace dense_depth_fusion
{
class DenseDepthFusion : public rclcpp::Node
{
public:
    DenseDepthFusion(const rclcpp::NodeOptions & options);

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2 & msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info_msg);
    void depth_image_callback(const sensor_msgs::msg::Image & msg);
    void synthesisDepth(const sensor_msgs::msg::PointCloud2 & pointcloud_msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_image_publisher_;

    cv::Mat depth_image_ ;
    sensor_msgs::msg::CameraInfo camera_info_;

    bool camera_info_init_;
    bool depth_img_init_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};
}
#endif // DENSE_DEPTH_FUSION_HPP
