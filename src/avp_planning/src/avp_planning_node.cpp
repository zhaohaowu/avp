#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>

#include "KinematicModel.h"
#include "avp_map.h"
#include "controller.h"
#include "lgsvl_msgs/CanBusData.h"
#include "lgsvl_msgs/VehicleControlData.h"
#include "planner.h"
#include "ros/ros.h"
#include "ros_viewer.h"
#include "visualization_msgs/Marker.h"

TimedPose startPose;

bool newGoalPoint = false;
Eigen::Vector2d goal(0, 0);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Received 2D Nav Goal:");
  ROS_INFO("Position: [x: %.2f, y: %.2f, z: %.2f]", msg->pose.position.x,
           msg->pose.position.y, msg->pose.position.z);
  ROS_INFO("Orientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
           msg->pose.orientation.x, msg->pose.orientation.y,
           msg->pose.orientation.z, msg->pose.orientation.w);

  newGoalPoint = true;
  goal.x() = msg->pose.position.x;
  goal.y() = msg->pose.position.y;
}

vector<double> robot_state(4);

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  robot_state[0] = msg->pose.pose.position.x;
  robot_state[1] = msg->pose.pose.position.y;
  robot_state[2] = tf2::getYaw(msg->pose.pose.orientation);

  startPose.time_ = 0;
  startPose.t_ =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
  startPose.R_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

void chassisCallback(const lgsvl_msgs::CanBusData::ConstPtr &msg) {
  robot_state[3] = msg->speed_mps;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_planning");
  ros::NodeHandle nh("~");
  ros::Subscriber sub_odometry =
      nh.subscribe("/avp/odometry", 10, &odometryCallback);
  ros::Subscriber sub1 =
      nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
  ros::Subscriber sub2 = nh.subscribe("/chassis", 10, &chassisCallback);
  ros::Publisher pub_vel =
      nh.advertise<lgsvl_msgs::VehicleControlData>("/vehicle_cmd", 10);

  AvpMap avp_map;
  avp_map.load(DATASET_PATH "avp_map.bin");
  RosViewer rosViewer(nh);
  // visualize avp map
  rosViewer.displayAvpMap(avp_map, 0);

  // get obstacles
  std::vector<Eigen::Vector3d> obstaclePoints;
  for (const auto &p : avp_map.getSemanticElement((SemanticLabel::kSlot))) {
    Eigen::Vector3d tmp_p(p.x() * kPixelScale, p.y() * kPixelScale,
                          p.z() * kPixelScale);
    obstaclePoints.push_back(tmp_p);
  }

  double resolution = 0.5;

  unordered_set<Vector2i, Vector2iHash> obstacles_index;
  for (auto &p : obstaclePoints) {
    Vector2i obs_p(round(p.x() / resolution), round(p.y() / resolution));
    obstacles_index.insert(obs_p);
  }

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();

    if (!newGoalPoint)
      continue;

    Vector2i start_index(round(startPose.t_(0) / resolution),
                         round(startPose.t_(1) / resolution));
    // newGoalPoint = false;
    Vector2i goal_index(round(goal.x() / resolution),
                        round(goal.y() / resolution));

    // find path
    Planner planner;
    // // BFS
    // vector<Vector2i> path = planner.Bfs(start_index, goal_index,
    // obstacles_index);
    // // publish result
    // if (path.empty()) {
    //     ROS_WARN("Fail to find BFS path");
    // } else {
    //     std::vector<Eigen::Vector2d> path_points;
    //     for (auto& index : path) {
    //         Eigen::Vector2d tmp(index.x() * resolution, index.y() *
    //         resolution); path_points.emplace_back(tmp);
    //     }
    //     rosViewer.publishTrajectory(path_points);
    // }

    // A*
    // vector<Vector2i> pathA = planner.AStar(start_index, goal_index,
    // obstacles_index);
    // // publish result
    // if (pathA.empty()) {
    //     ROS_WARN("Fail to find A* path");
    //     continue;
    // } else {
    //     std::vector<Eigen::Vector2d> path_points;
    //     for (auto& index : pathA) {
    //         Eigen::Vector2d tmp(index.x() * resolution, index.y() *
    //         resolution); path_points.emplace_back(tmp);
    //     }
    //     rosViewer.publishATrajectory(path_points);
    // }

    // // 车辆参数  for hybrid A*
    double wheelbase = 2.5;      // 轴距
    double step_size = 0.5;      // 步长
    double max_steer = M_PI / 4; // 最大转向角（45度）
    State startState(startPose.t_(0), startPose.t_(1), M_PI / 2, 0, 0, nullptr);
    State goalState(goal.x(), goal.y(), 0, 0, 0, nullptr);
    std::vector<State *> hybridApath =
        planner.HybridAStar(startState, goalState, obstacles_index, wheelbase,
                            step_size, max_steer);
    // publish result
    if (hybridApath.empty()) {
      ROS_WARN("Fail to find Hybrid A* path");
    } else {
      std::vector<Eigen::Vector2d> path_points;
      for (auto &pState : hybridApath) {
        Eigen::Vector2d tmp(pState->x, pState->y);
        path_points.emplace_back(tmp);
      }
      rosViewer.publishHybridATrajectory(path_points);
    }

    // 显示定位
    rosViewer.publishPose(startPose);

    // 控制
    Controller controller;
    vector<vector<double>> refer_path;
    for (const auto &p : hybridApath) {
      vector<double> tmp_p;
      tmp_p.push_back(p->x);
      tmp_p.push_back(p->y);
      refer_path.emplace_back(tmp_p);
    }
    KinematicModel vehicle_model(0, 0, 0, 2, 2.5, 0.1);

    ControlOutput control_output =
        controller.PIDController(robot_state, refer_path, vehicle_model);

    lgsvl_msgs::VehicleControlData control_msg;
    control_msg.header.stamp = ros::Time::now();
    control_msg.header.frame_id = "world";
    control_msg.acceleration_pct = control_output.accel_pct;
    control_msg.target_wheel_angle = control_output.steer_angle;
    control_msg.braking_pct = control_output.brake_pct;
    std::cout << "accel_pct: " << control_output.accel_pct << std::endl;
    std::cout << "steer_angle: " << control_output.steer_angle << std::endl;
    std::cout << "brake_pct: " << control_output.brake_pct << std::endl;

    pub_vel.publish(control_msg);
  }
}