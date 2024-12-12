#include "dense_depth_fusion/dense_depth_fusion_component.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<dense_depth_fusion::DenseDepthFusion>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}