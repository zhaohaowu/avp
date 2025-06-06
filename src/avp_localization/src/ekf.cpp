#include "ekf.h"
#include <iostream>

namespace {

State interpolation(const State& start, const State& end,
                        double time) {
  double factor = (time - start.time) / (end.time - start.time);
  State ret;
  ret.time = time;
  ret.x = start.x + factor * (end.x - start.x);
  ret.y = start.y + factor * (end.y - start.y);
  ret.yaw = start.yaw + factor * (end.yaw - start.yaw);
  ret.P = start.P + factor * (end.P - start.P);
  
  return ret;
}

}  // namespace

void StateInterpolation::Push(const State& pose) {
  if (pose.time > EarliestTime()) {
    poses_.push_back(pose);
  }
}

void StateInterpolation::TrimBefore(double time) {
  while (!poses_.empty() && poses_.front().time < time) {
    poses_.pop_front();
  }
}

double StateInterpolation::EarliestTime() const {
  return poses_.empty() ? 0. : poses_.front().time;
}

bool StateInterpolation::LookUp(State& pose) const {
  if (poses_.empty() || pose.time < poses_.front().time ||
      pose.time > poses_.back().time) {
    if (poses_.empty()){
      std::cout << "poses empty!" << std::endl;
    }else{
      std::cout << std::to_string(pose.time) << ", poses have: " << std::to_string(poses_.front().time)
                << " -- " << std::to_string(poses_.back().time) << std::endl;
    }
    return false;
  }

  auto end = std::lower_bound(
      poses_.begin(), poses_.end(), pose.time,
      [](const State& _pose, const double t) { return _pose.time < t; });

  if (end == poses_.end()) {
    end = std::prev(end);
  }
  pose = interpolation(*std::prev(end), *end, pose.time);
  return true;
}


EKF::EKF(double time, double x, double y, double yaw)
{
    // variance of motion noise [v, yaw_rate]
    Qn << v_std * v_std, 0,
        0, yaw_rate_std * yaw_rate_std;

    // variance of match noise [x, y, yaw]
    Rn_match << x_match_std * x_match_std, 0, 0,
        0, y_match_std * y_match_std, 0,
        0, 0, yaw_match_std * yaw_match_std;


    // initialize state_
    state_.x = x;
    state_.y = y;
    state_.yaw = yaw;
    state_.P = 10 * 10 * Eigen::Matrix3d::Identity();
    state_.time = time;
    init_ = true;
    state_interpolation_.Push(state_);
}


void EKF::ekfPredict(const double &time, const double &velocity, const double &yaw_rate)
{
    double dt = time - state_.time;
    if (dt == 0)
        return;

    // car heading y-axis in vehicle frame,velocity direction is y-axis
    Eigen::Matrix3d A;
    A << 0, 0, -velocity * sin(state_.yaw + M_PI_2),
        0, 0, velocity * cos(state_.yaw + M_PI_2),
        0, 0, 0;
    Eigen::Matrix<double, 3, 2> U;
    U << cos(state_.yaw), 0,
        sin(state_.yaw), 0,
        0, 1;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity() + dt * A;
    Eigen::Matrix<double, 3, 2> V;
    V = dt * U;

    state_.x += dt * velocity * cos(state_.yaw + M_PI_2);
    state_.y += dt * velocity * sin(state_.yaw + M_PI_2);
    state_.yaw += dt * yaw_rate;
    if (state_.yaw > M_PI)
    {
        state_.yaw -= 2 * M_PI;
    }
    if (state_.yaw < -M_PI)
    {
        state_.yaw += 2 * M_PI;
    }

    state_.P = F * state_.P * F.transpose() + V * Qn * V.transpose();
    state_.time = time;

    state_interpolation_.Push(state_);

    // printf("     -- input : velocity %.3f, yaw_rate %.2f \n", velocity, yaw_rate * 57.3);
    // printf("Predict t = %.4f, x: %.3f, y: %.3f, yaw: %.2f \n",state_.time, state_.x, state_.y, state_.yaw * 57.3);
}

void EKF::ekfUpdate(const double &m_x, const double &m_y, const double &m_yaw)
{
    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd K = state_.P * C.transpose() * (C * state_.P * C.transpose() + W * Rn_match * W.transpose()).inverse();

    Eigen::Vector3d r;
    r(0) = m_x - state_.x;
    r(1) = m_y - state_.y;
    r(2) = m_yaw - state_.yaw;
    if (r(2) > M_PI)
    {
        r(2) -= 2 * M_PI;
    }
    if (r(2) < -M_PI)
    {
        r(2) += 2 * M_PI;
    }

    Eigen::Vector3d correct = K * r;

    state_.x += correct(0);
    state_.y += correct(1);
    state_.yaw += correct(2);

    state_.P += -K * C * state_.P;

    printf("Update t = %.4f, x: %.3f, y: %.3f, yaw: %.2f \n", state_.time, state_.x, state_.y, state_.yaw * 57.3);
}