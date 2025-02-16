cmake_minimum_required(VERSION 3.8)
project(dense_depth_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(dense_depth_fusion_component SHARED
 src/dense_depth_fusion_component.cpp)

rclcpp_components_register_nodes(dense_depth_fusion_component
  "dense_depth_fusion::DenseDepthFusion"
)

ament_auto_add_executable(dense_depth_fusion_node
    NO_TARGET_LINK_LIBRARIES
    src/dense_depth_fusion_node.cpp
)

target_link_libraries(dense_depth_fusion_node
dense_depth_fusion_component
)

# find dependencies
install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package()