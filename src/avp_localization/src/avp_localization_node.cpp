#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include "avp_localization.h"
#include "lgsvl_msgs/CanBusData.h"
#include "map.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

ros::Publisher avp_odometry_pub;

struct Node {
    Node() { avp_localization_.reset(new AvpLocalization(DATASET_PATH "avp_map.bin")); }

    void addOdomGps(const nav_msgs::OdometryConstPtr& msg) {
        Eigen::Affine3d T_w_v = Eigen::Affine3d::Identity();
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        t.x() = msg->pose.pose.position.x;
        t.y() = msg->pose.pose.position.y;
        t.z() = msg->pose.pose.position.z;
        q.x() = msg->pose.pose.orientation.x;
        q.y() = msg->pose.pose.orientation.y;
        q.z() = msg->pose.pose.orientation.z;
        q.w() = msg->pose.pose.orientation.w;

        T_w_v.linear() = q.toRotationMatrix();
        T_w_v.translation() = t;
        static Eigen::Affine3d T_first = T_w_v;
        // T_w_v = T_first.inverse() * T_w_v;
        q = T_w_v.linear();
        t = T_w_v.translation();

        if (!ekf_inited_) {  // set ref pose
            ekf_inited_ = true;
            // add errors to init
            avp_localization_->initState(msg->header.stamp.toSec(), t.x(), t.y(), GetYaw(q));
        }
        // printf("gps_gt: t = %.4f, x= %.3f , y = %.3f, yaw = %.2f \n",
        //        msg->header.stamp.toSec(), t.x(), t.y(), kToDeg * GetYaw(q));
    }

    void addImuSpeed(const sensor_msgs::ImuConstPtr& imu, const lgsvl_msgs::CanBusDataConstPtr& speed) {
        if (ekf_inited_) {
            WheelMeasurement wheel_measurement;
            wheel_measurement.time_ = imu->header.stamp.toSec();
            wheel_measurement.velocity_ = speed->speed_mps;
            wheel_measurement.yaw_rate_ = imu->angular_velocity.z;
            avp_localization_->processWheelMeasurement(wheel_measurement);
        }
    }

    void addIPmImage(const sensor_msgs::CompressedImageConstPtr& msg) {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!image.empty()) {
            double cur_time = msg->header.stamp.toSec();
            avp_localization_->processImage(cur_time, image);
        }
        if (ekf_inited_) {
            State state = avp_localization_->getState();
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time(state.time);
            msg.header.frame_id = "world";
            msg.child_frame_id = "base_link";
            msg.pose.pose.position.x = state.x;
            msg.pose.pose.position.y = state.y;
            msg.pose.pose.position.z = 0;
            Eigen::Quaterniond q =
                Eigen::Quaterniond(Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ())).normalized();
            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            avp_odometry_pub.publish(msg);
        }
    }

    bool ekf_inited_{false};
    std::shared_ptr<AvpLocalization> avp_localization_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "avp_localization_node");
    ros::NodeHandle nh("~");

    Node node;

    message_filters::Subscriber<sensor_msgs::Imu> sub0(nh, "/imu", 100);
    message_filters::Subscriber<lgsvl_msgs::CanBusData> sub1(nh, "/chassis", 100);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Imu, lgsvl_msgs::CanBusData> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), sub0, sub1);
    sync.registerCallback(boost::bind(&Node::addImuSpeed, &node, _1, _2));

    ros::Subscriber gps_sub =
        nh.subscribe<nav_msgs::Odometry>("/gps_odom", 100, boost::bind(&Node::addOdomGps, &node, _1));
    ros::Subscriber ipm_sub =
        nh.subscribe<sensor_msgs::CompressedImage>("/ipm_label", 10, boost::bind(&Node::addIPmImage, &node, _1));

    avp_odometry_pub = nh.advertise<nav_msgs::Odometry>("/avp/odometry", 10, true);

    ros::spin();
}