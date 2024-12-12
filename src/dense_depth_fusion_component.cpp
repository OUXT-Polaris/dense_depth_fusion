#include "dense_depth_fusion/dense_depth_fusion_component.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>

namespace dense_depth_fusion
{
DenseDepthFusion::DenseDepthFusion(const rclcpp::NodeOptions & options)
: Node("dense_depth_fusion", options),
  buffer_(get_clock()),
  listener_(buffer_)
{
    std::string pointcloud_topic;
    std::string camera_info_topic;
    std::string depth_image_topic;
    declare_parameter<std::string>("pointcloud_topic","/wamv/sensors/lidars/lidar_wamv_sensor/points"); 
    declare_parameter<std::string>("camera_info_topic","/wamv/sensors/cameras/ball_shooter_camera_sensor/camera_info"); 
    declare_parameter<std::string>("depth_image_topic","/depth_anything_v1/depth"); 
    get_parameter("pointcloud_topic", pointcloud_topic);
    get_parameter("camera_info_topic", camera_info_topic);
    get_parameter("depth_image_topic", depth_image_topic);

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, 10,[this](const sensor_msgs::msg::PointCloud2 & pointcloud_msg) {
        pointcloud_callback(pointcloud_msg);
    });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo & camera_info_msg) {
        camera_info_callback(camera_info_msg);
    });
    depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_image_topic, 10, [this](const sensor_msgs::msg::Image & depth_image_msg) {
        depth_image_callback(depth_image_msg);
    });
    fused_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/fused_depth_image", 10);
}

void DenseDepthFusion::pointcloud_callback(const sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
    if (!camera_info_init_ || !depth_img_init_)return;
    synthesisDepth(pointcloud_msg);
}

void DenseDepthFusion::depth_image_callback(const sensor_msgs::msg::Image & msg)
{
    if (msg.encoding != "mono8")
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg.encoding.c_str());
        return;
    }
    depth_image_ = cv_bridge::toCvCopy(msg, msg.encoding)->image;
    depth_img_init_ = true;  //TODO 初期化判定変える
}

void DenseDepthFusion::camera_info_callback(
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  camera_info_ = camera_info_msg;
  camera_info_init_ = true;　//TODO 初期化判定変える
}

//最近帽の画像画像
void DenseDepthFusion::synthesisDepth(const sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    try {
        transform_stamped = buffer_.lookupTransform(
        camera_info_.header.frame_id, pointcloud_msg.header.frame_id, camera_info_.header.stamp,
        rclcpp::Duration::from_seconds(0.1));
        //点群::LiDAR座標系⇨カメラ行列
        tf2::doTransform(pointcloud_msg, transformed_pointcloud, transform_stamped);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
        return;
    }

    Eigen::Matrix4d camera_projection;  //Homogeneous Coordinates
    camera_projection << camera_info_.p.at(0), camera_info_.p.at(1), camera_info_.p.at(2),
    camera_info_.p.at(3), camera_info_.p.at(4), camera_info_.p.at(5), camera_info_.p.at(6),
    camera_info_.p.at(7), camera_info_.p.at(8), camera_info_.p.at(9), camera_info_.p.at(10),
    camera_info_.p.at(11), 0.0, 0.0, 0.0, 1.0;

    float sum_squared_errors = 0;
    int projected_points_cnt = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
        iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Eigen::Vector4d projected_point =
        camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
        Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
        projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
        if (projected_point.z() < 0.0) {
         continue;
        }
        int img_point_x = int(normalized_projected_point.x());
        int img_point_y = int(normalized_projected_point.y());
        if (img_point_x >= 0 && img_point_x < static_cast<int>(camera_info_.height) &&
             img_point_y >= 0 && img_point_y < static_cast<int>(camera_info_.width)) 
        {
            if (!std::isfinite(*iter_z) || *iter_z <= 0.0f) {
                continue; 
            }
            // RCLCPP_INFO(this->get_logger(),"LiDAR depth value: %s", std::to_string(*iter_z).c_str());
            uint8_t pixel_value = depth_image_.at<uint8_t>(img_point_y, img_point_x);
            if (pixel_value > 0) {  
                // RCLCPP_INFO(this->get_logger(),"iter_z : %s", std::to_string((*iter_z)*1000.0).c_str());
                sum_squared_errors += ((*iter_z)*1000.0) / static_cast<double>(pixel_value);
                projected_points_cnt++;
            }
        }
    }
    if (projected_points_cnt == 0) {
        RCLCPP_WARN(this->get_logger(), "No projected points in the image.");
        return;
    }

    double distance_ratio = sum_squared_errors / projected_points_cnt;
    RCLCPP_INFO(this->get_logger(),"distance_ratio : %s", std::to_string(distance_ratio).c_str());
    cv::Mat depth_mono16_image_;
    depth_image_.convertTo(depth_mono16_image_, CV_16U, distance_ratio);

    //publish dense depth
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = camera_info_.header.frame_id;
    sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(header, "mono16", depth_mono16_image_).toImageMsg();
    fused_image_publisher_->publish(*ros_image);
    // RCLCPP_INFO(this->get_logger(), "Published fused depth image.");
}
} //dense_depth_fusion
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dense_depth_fusion::DenseDepthFusion)
