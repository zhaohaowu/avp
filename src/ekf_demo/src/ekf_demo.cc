//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <random>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_estimation_path, pub_gt_path, pub_measurement_path,
    pub_odometry;
nav_msgs::Path path_estimation, path_gt, path_measurment;
ros::Publisher meshPub;

// parameters for noise
double v_std = 0.5;
double yaw_rate_std = 0.1;
double x_std = 0.5;
double y_std = 0.5;
double yaw_std = 5.0 / 180. * M_PI;

// guassian noise generator
std::default_random_engine generator;
std::normal_distribution<double> dist_x(0, x_std);
std::normal_distribution<double> dist_y(0, y_std);
std::normal_distribution<double> dist_yaw(0, yaw_std);
std::normal_distribution<double> dist_v(0, v_std);
std::normal_distribution<double> dist_yaw_rate(0, yaw_rate_std);

// matrix for noise
Eigen::Matrix2d Qn;
Eigen::Matrix3d Rn;

struct State {
  double time;
  double x;
  double y;
  double yaw;
  Eigen::Matrix3d P;
};

void PublishPath(const double &time, const double &x, const double &y,
                 const double &yaw, nav_msgs::Path &path,
                 ros::Publisher &path_publisher) {
  // convert 2D x, y, yaw to 3D x, y, z and rotation matrix
  Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
  Eigen::Matrix3d R;
  R << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Eigen::Quaterniond q(R);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "world";
  odometry.header.stamp = ros::Time(time);
  odometry.pose.pose.position.x = position(0);
  odometry.pose.pose.position.y = position(1);
  odometry.pose.pose.position.z = position(2);
  odometry.pose.pose.orientation.x = q.x();
  odometry.pose.pose.orientation.y = q.y();
  odometry.pose.pose.orientation.z = q.z();
  odometry.pose.pose.orientation.w = q.w();

  // pub path
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  pose_stamped.header.stamp = ros::Time(time);
  pose_stamped.pose = odometry.pose.pose;
  path.poses.push_back(pose_stamped);
  if (path.poses.size() > 10000) {
    path.poses.erase(path.poses.begin());
  }
  path_publisher.publish(path);
}

void PublishPathAndTF(const double &time, const double &x, const double &y,
                      const double &yaw, nav_msgs::Path &path,
                      ros::Publisher &path_publisher) {
  PublishPath(time, x, y, yaw, path, path_publisher);
  // convert 2D x, y, yaw to 3D x, y, z and rotation matrix
  Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
  Eigen::Matrix3d R;
  R << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Eigen::Quaterniond q(R);

  // pub tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion tf_q;
  transform.setOrigin(tf::Vector3(position(0), position(1), position(2)));
  tf_q.setW(q.w());
  tf_q.setX(q.x());
  tf_q.setY(q.y());
  tf_q.setZ(q.z());
  transform.setRotation(tf_q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time(time), "world", "vehicle"));

  // pub Mesh model
  visualization_msgs::Marker meshROS;
  meshROS.header.frame_id = std::string("world");
  meshROS.header.stamp = ros::Time(time);
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  Eigen::Matrix3d rot_mesh;
  rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
  Eigen::Quaterniond q_mesh;
  q_mesh = q * rot_mesh;
  Eigen::Vector3d t_mesh = R * Eigen::Vector3d(1.5, 0, 0) + position;
  meshROS.pose.orientation.w = q_mesh.w();
  meshROS.pose.orientation.x = q_mesh.x();
  meshROS.pose.orientation.y = q_mesh.y();
  meshROS.pose.orientation.z = q_mesh.z();
  meshROS.pose.position.x = t_mesh(0);
  meshROS.pose.position.y = t_mesh(1);
  meshROS.pose.position.z = t_mesh(2);
  meshROS.scale.x = 1.0;
  meshROS.scale.y = 1.0;
  meshROS.scale.z = 1.0;
  meshROS.color.a = 1.0;
  meshROS.color.r = 1.0;
  meshROS.color.g = 0.0;
  meshROS.color.b = 0.0;
  std::string mesh_resource = "package://ekf_demo/meshes/car.dae";
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);
}

void GeneratePose(const double &time, double &x, double &y, double &yaw) {
  x = 10 * sin(time / 10 * M_PI);
  y = 10 * cos(time / 10 * M_PI) - 10;
  double vx = (1.0 / 10 * M_PI) * 10 * cos(time / 10 * M_PI);
  double vy = (1.0 / 10 * M_PI) * 10 * (-1) * sin(time / 10 * M_PI);
  yaw = atan2(vy, vx);
}

void EkfPredict(State &state, const double &time, const double &velocity,
                const double &yaw_rate) {
  // printf("time %lf, velocity %lf, yaw_rate %lf \n", time, velocity,
  // yaw_rate); YOUR_CODE_HERE todo: implement the EkfPredict function
  float dt = time - state.time;
  state.time = time;
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -velocity * sin(state.yaw) * dt;
  F(1, 2) = velocity * cos(state.yaw) * dt;
  state.x += velocity * cos(state.yaw) * dt;
  state.y += velocity * sin(state.yaw) * dt;
  state.yaw += yaw_rate * dt;
  Eigen::Matrix<double, 3, 2> V = Eigen::Matrix<double, 3, 2>::Zero();
  V(0, 0) = cos(state.yaw) * dt;
  V(1, 0) = sin(state.yaw) * dt;
  V(2, 1) = dt;
  state.P = F * state.P * F.transpose() + V * Qn * V.transpose();
  // printf("after predict x: %lf, y: %lf, yaw: %lf \n", state.x, state.y,
  // state.yaw);
}

void EkfUpdate(State &state, const double &m_x, const double &m_y,
               const double &m_yaw) {
  // printf("time :%lf \n", state.time);
  // printf("before update x: %lf, y: %lf, yaw: %lf \n", state.x, state.y,
  // state.yaw); printf("measure x: %lf, y: %lf, yaw: %lf \n", m_x, m_y, m_yaw);
  // YOUR_CODE_HERE
  // todo: implement the EkfUpdate function
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K =
      state.P * H.transpose() * (H * state.P * H.transpose() + Rn).inverse();
  Eigen::Vector3d prior_x{state.x, state.y, state.yaw};
  Eigen::Vector3d measure_z{m_x, m_y, m_yaw};
  Eigen::Vector3d diff = measure_z - prior_x;
  diff.z() = atan2(sin(diff.z()), cos(diff.z())); // 确保观测和预测差异不能过大
  prior_x = prior_x + K * diff;
  state.x = prior_x.x();
  state.y = prior_x.y();
  state.yaw = atan2(sin(prior_x.z()), cos(prior_x.z()));
  state.P = (Eigen::Matrix3d::Identity() - K * H) * state.P;

  // printf("after update x: %lf, y: %lf, yaw: %lf \n", state.x, state.y,
  // state.yaw);
}

void GetMotionSignal(const double &time, double &velocity, double &yaw_rate) {
  double x0, y0, yaw0;
  double t1 = time + 1e-6;
  double x1, y1, yaw1;
  GeneratePose(time, x0, y0, yaw0);
  GeneratePose(t1, x1, y1, yaw1);
  double vx = (x1 - x0) / 1e-6;
  double vy = (y1 - y0) / 1e-6;
  double dyaw = yaw1 - yaw0;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  if (dyaw < -M_PI)
    dyaw += 2 * M_PI;
  yaw_rate = dyaw / 1e-6;
  velocity = sqrt(vx * vx + vy * vy);

  // add Gaussian noise
  velocity += dist_v(generator);
  yaw_rate += dist_yaw_rate(generator);
}

void GetPoseMeasurement(const double &time, double &mx, double &my,
                        double &myaw) {
  GeneratePose(time, mx, my, myaw);

  // add Gaussian noise
  mx += dist_x(generator);
  my += dist_y(generator);
  myaw += dist_yaw(generator);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ekf_demo");
  ros::NodeHandle n("~");

  pub_estimation_path = n.advertise<nav_msgs::Path>("path_estimation", 1000);
  pub_gt_path = n.advertise<nav_msgs::Path>("path_gt", 1000);
  pub_measurement_path = n.advertise<nav_msgs::Path>("path_measurement", 1000);
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  meshPub = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
  path_estimation.header.frame_id = "world";
  path_gt.header.frame_id = "world";
  path_measurment.header.frame_id = "world";

  // variance of motion noise [v, yaw_rate]
  Qn << v_std * v_std, 0, 0, yaw_rate_std * yaw_rate_std;

  // variance of measurement noise [x, y, yaw]
  Rn << x_std * x_std, 0, 0, 0, y_std * y_std, 0, 0, 0, yaw_std * yaw_std;

  // initialize state
  State state;
  state.x = 0;
  state.y = 0;
  state.yaw = 0;
  state.P = 10 * 10 * Eigen::Matrix3d::Identity();
  state.time = 0;

  ros::Rate loop_rate(100);
  double time = 0;
  int cnt = 0;
  while (ros::ok()) {
    // predict at 100hz
    double velocity, yaw_rate = 0;
    GetMotionSignal(time, velocity, yaw_rate);
    // Ekf predict
    EkfPredict(state, time, velocity, yaw_rate);

    // update at 10hz
    if (cnt % 10 == 0) {
      double mx, my, myaw = 0;
      GetPoseMeasurement(time, mx, my, myaw);
      // publish raw measurement
      PublishPath(time, mx, my, myaw, path_measurment, pub_measurement_path);
      // Ekf update
      EkfUpdate(state, mx, my, myaw);
    }

    // publish estimation result
    PublishPathAndTF(time, state.x, state.y, state.yaw, path_estimation,
                     pub_estimation_path);

    // generate and publish ground truth
    double x_gt = 0, y_gt = 0, yaw_gt = 0;
    GeneratePose(time, x_gt, y_gt, yaw_gt);
    PublishPath(time, x_gt, y_gt, yaw_gt, path_gt, pub_gt_path);

    ros::spinOnce();
    loop_rate.sleep();

    time += 0.01;
    cnt++;
  }
  ros::spin();
}
