#pragma once
#include <deque>
#include <eigen3/Eigen/Geometry>
#include <random>

// parameters for noise
constexpr double v_std = 1e-2;
constexpr double yaw_rate_std = 1e-8;

// parameters for match
constexpr double x_match_std = 0.3;
constexpr double y_match_std = 0.3;
constexpr double yaw_match_std = 10.0 / 180. * M_PI;

struct State {
  double time;
  double x;
  double y;
  double yaw;
  Eigen::Matrix3d P;
};

class StateInterpolation {
 public:
  StateInterpolation() = default;
  void Push(const State &pose);
  void TrimBefore(double time);
  double EarliestTime() const;
  bool LookUp(State &pose) const;

 private:
  std::deque<State> poses_;
};

class EKF {
 public:
  EKF() = default;
  EKF(double time, double x, double y, double yaw);

  void ekfPredict(const double &time, const double &velocity,
                  const double &yaw_rate);

  void ekfUpdate(const double &m_x, const double &m_y, const double &m_yaw);

  State getState() const { return state_; }
  bool isInit() const { return init_; }
  bool init_{false};
  State state_;
  StateInterpolation state_interpolation_;

  Eigen::Matrix2d Qn;
  Eigen::Matrix3d Rn_match;
};
