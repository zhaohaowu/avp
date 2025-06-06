#ifndef AVP_WS_CONTROLLER_H
#define AVP_WS_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "KinematicModel.h"

using namespace std;
#define PI 3.1415926

struct ControlOutput {
    double steer_angle;  // 横向控制量（方向盘转角，单位：弧度）
    double accel_pct;    // 纵向控制量（油门百分比，0.0~1.0）
    double brake_pct;    // 纵向控制量（刹车百分比，0.0~1.0）
};

class Controller {
public:
    double calTargetIndex(const vector<double>& robot_state, const vector<vector<double>>& refer_path);

    double calTargetIndex(vector<double> robot_state, vector<vector<double>> refer_path, double l_d);

    double PIDController(const vector<double>& robot_state, const vector<vector<double>>& refer_path);

    ControlOutput PIDController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                const KinematicModel& ugv);

    double PurePursuitController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                 const KinematicModel& ugv);

    double StanlyController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                            const KinematicModel& ugv);

    double LQRController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                         const KinematicModel& ugv);
};

#endif  // AVP_WS_CONTROLLER_H
