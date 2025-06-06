#include <iostream>

#include <eigen3/Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include "ipm.h"
#include "tic_tok.h"

std::vector<cv::Mat> label_images(4);

std::shared_ptr<sensor_msgs::CompressedImage> front_seg = nullptr;
std::shared_ptr<sensor_msgs::CompressedImage> back_seg = nullptr;
std::shared_ptr<sensor_msgs::CompressedImage> left_seg = nullptr;
std::shared_ptr<sensor_msgs::CompressedImage> right_seg = nullptr;

void CameraFrontSeg(const sensor_msgs::CompressedImage &msg) {
  front_seg = std::make_shared<sensor_msgs::CompressedImage>(msg);
}
void CameraBackSeg(const sensor_msgs::CompressedImage &msg) {
  back_seg = std::make_shared<sensor_msgs::CompressedImage>(msg);
}
void CameraLeftSeg(const sensor_msgs::CompressedImage &msg) {
  left_seg = std::make_shared<sensor_msgs::CompressedImage>(msg);
}
void CameraRightSeg(const sensor_msgs::CompressedImage &msg) {
  right_seg = std::make_shared<sensor_msgs::CompressedImage>(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ipm_node");
  ros::NodeHandle nh("~");
  ros::Subscriber sub_camera_front_seg =
      nh.subscribe("/camera/front_seg", 10, CameraFrontSeg);
  ros::Subscriber sub_camera_back_seg =
      nh.subscribe("/camera/back_seg", 10, CameraBackSeg);
  ros::Subscriber sub_camera_left_seg =
      nh.subscribe("/camera/left_seg", 10, CameraLeftSeg);
  ros::Subscriber sub_camera_right_seg =
      nh.subscribe("/camera/right_seg", 10, CameraRightSeg);
  ros::Publisher pub_ipm_label =
      nh.advertise<sensor_msgs::CompressedImage>("/ipm_label", 10);
  IPM ipm_label;
  for (int i = 0; i < 4; ++i) {
    std::string ex_file(PARAMS + std::to_string(i) + "_extrinsic.yaml");
    std::string in_file(PARAMS + std::to_string(i) + "_intrinsic.yaml");
    ipm_label.AddCamera(in_file, ex_file);
  }
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    TicToc t;
    ros::spinOnce();
    double cur_time;
    if (front_seg && back_seg && left_seg && right_seg &&
        fabs(front_seg->header.stamp.sec - back_seg->header.stamp.sec) < 0.03 &&
        fabs(front_seg->header.stamp.sec - left_seg->header.stamp.sec) < 0.03 &&
        fabs(front_seg->header.stamp.sec - right_seg->header.stamp.sec) <
            0.03) {
      label_images[0] = cv::imdecode(cv::Mat(front_seg->data), 1);
      label_images[1] = cv::imdecode(cv::Mat(back_seg->data), 1);
      label_images[2] = cv::imdecode(cv::Mat(left_seg->data), 1);
      label_images[3] = cv::imdecode(cv::Mat(right_seg->data), 1);
      cur_time = front_seg->header.stamp.toSec();
      front_seg = nullptr;
      back_seg = nullptr;
      left_seg = nullptr;
      right_seg = nullptr;
    } else {
      std::cout << "failed" << std::endl;
      loop_rate.sleep();
      continue;
    }

    std::cout << "sync " << t.toc() << "ms" << std::endl;
    cv::Mat ipm_img_label = ipm_label.GenerateIPMImage(label_images);
    std::cout << "generate " << t.toc() << "ms" << std::endl;
    cv::namedWindow("ipm_img_label", cv::WINDOW_NORMAL);
    cv::imshow("ipm_img_label", ipm_img_label);
    cv::waitKey(1);

    sensor_msgs::CompressedImage ipm_label;
    ipm_label.header.stamp = ros::Time(cur_time);
    ipm_label.format = "jpeg";
    std::vector<int> params_label;
    params_label.push_back(cv::IMWRITE_JPEG_QUALITY);
    params_label.push_back(90);
    cv::imencode(".jpg", ipm_img_label, ipm_label.data, params_label);
    pub_ipm_label.publish(ipm_label);
    loop_rate.sleep();
    std::cout << "total time: " << t.toc() << "ms" << std::endl;
  }
  return 0;
}