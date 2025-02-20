## Dense_Depth_Fusion
Create density depth information from relative distance depth images and Lidar depth information

# subscribe
・`pointcloud_topic`  :　LiDAR data  
・`camera_info_topic` :　Camera meta information  
・`depth_image_topic` :　Relative depth image
# publish
・`dense_depth_publisher_` :　Absolute Depth Images

# parameters
|Name|Type|Description|default|
|---|---|---|---|
|pointcloud_topic |string|point cloud topic name||
|camera_info_topic|string|cam info topic name||
|depth_image_topic|string|depth img topic name||



# launch 
```
ros2 launch dense_depth_fusion dense_depth_fusion_launch.xml
```

## demo  
[rosbag Download link](https://drive.google.com/drive/folders/1A9gW5Q-j6wgjgRvzeZ26p-IUtJY9Iu4u?usp=sharing)

```bash
# ros2 bag play(relative depth rosbag data)
cd depth_anything_data
ros2 bag play ros2 bag play depth_anything_data_0.mcap

#In other terminals
cd (your_ws)
source install/setup.bash
ros2 launch dense_depth_fusion dense_depth_fusion_launch.xml
```

[Screencast from 12-11-2024 10:35:15 PM.webm](https://github.com/user-attachments/assets/d6c10fa9-9ae9-4d96-b98f-fee4c6cd3889)


# Visualization of results

https://github.com/OUXT-Polaris/depthimage_to_pointcloud2
```
```
