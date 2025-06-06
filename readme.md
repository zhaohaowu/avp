## 1、依赖

1. ROS https://wiki.ros.org/cn/noetic/Installation/Ubuntu
2. PCL
3. OpenCV
4. Eigen

## 2、编译

```
cd avp && catkin_make
```

## 3、数据集链接&&各章节作业&&PPT

https://e1cdeqo30q.feishu.cn/docx/OUtrdn2XEo1qNhx5rICcCiy3n5d

## 4、运行

```xml
# 1. 融合imu、chassis与gps的ekf算法
roslaunch ekf_demo ekf_demo.launch

# 2. 鱼眼相机拼接俯视图
roslaunch ipm ipm.launch

# 3. 使用俯视图构建语义图
roslaunch avp_mapping avp_mapping.launch

# 4. 基于语义图定位与imu、chassis使用ekf融合
roslaunch avp_localization avp_localization.launch

# 5. 基于语义地图和定位信息规划（bfs、a*和混合a*）和控制（pid、stanly、纯跟踪和lqr）
roslaunch avp_planning avp_planning.launch

# 播放数据集
rosbag play 2025-04-15-15-23-25.bag
```
