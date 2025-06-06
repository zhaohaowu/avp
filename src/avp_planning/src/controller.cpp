#include "controller.h"

#include "ros/ros.h"

/**
 * find the index of the point on the reference path which is closest to robot_state
 * @param robot_state robot state（x,y）
 * @param refer_path  reference path
 * @return the index of the closest point
 */
#define EPS 1.0e-4

double Controller::calTargetIndex(const vector<double>& robot_state, const vector<vector<double>>& refer_path) {
    vector<double> dists;
    for (vector<double> xy : refer_path) {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(), dists.end()) - dists.begin();  // 返回vector最小元素的下标
}

double Controller::calTargetIndex(vector<double> robot_state, vector<vector<double>> refer_path, double l_d) {
    vector<double> dists;
    for (vector<double> xy : refer_path) {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    double min_ind = min_element(dists.begin(), dists.end()) - dists.begin();  // 返回vector最小元素的下标

    double delta_l =
        sqrt(pow(refer_path[min_ind][0] - robot_state[0], 2) + pow(refer_path[min_ind][1] - robot_state[1], 2));

    while (l_d > delta_l && min_ind < refer_path.size() - 1) {
        delta_l = sqrt(pow(refer_path[min_ind + 1][0] - robot_state[0], 2) +
                       pow(refer_path[min_ind + 1][1] - robot_state[1], 2));
        min_ind += 1;
    }
    return min_ind;
}

double normalizeAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

MatrixXd CalRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
    MatrixXd Qf = Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for (int i = 0; i < 100; i++) {
        P_ = Q + A.transpose() * P * A -
             A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if ((P_ - P).maxCoeff() < EPS && (P - P_).maxCoeff() < EPS) break;
        P = P_;
    }
    return P_;
}

// 控制量结构体（横向转向 + 纵向油门/刹车）
ControlOutput Controller::PIDController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                        const KinematicModel& model) {
    ControlOutput output;
    output.accel_pct = 0.0;
    output.brake_pct = 0.0;
    output.steer_angle = 0.0;

    if (refer_path.empty()) return output;

    // 控制参数 (已适配Y轴前进坐标系)
    const double target_speed = 1.5;  // m/s
    const double kp_steer = 0.3;      // 降低转向增益
    const double kd_steer = 0.2;
    const double lookahead_base = 3.0;          // 增大前视距离
    const double max_steer_angle = M_PI / 6.0;  // 最大30度转向

    // 当前状态
    const double current_x = robot_state[0];    // 全局X（车辆右侧）
    const double current_y = robot_state[1];    // 全局Y（车辆前方）
    const double current_yaw = robot_state[2];  // 航向角（全局坐标系，0弧度指向全局Y+）
    const double current_speed = robot_state[3];

    // 1. 寻找最近路径点
    int closest_index = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < refer_path.size(); ++i) {
        const double dx = refer_path[i][0] - current_x;
        const double dy = refer_path[i][1] - current_y;
        const double dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    // 2. 动态前视距离计算（基于Y轴速度）
    const double lookahead_dist = lookahead_base + 0.5 * current_speed;
    int target_index = closest_index;
    double accumulated_dist = 0.0;
    while (target_index < refer_path.size() - 1 && accumulated_dist < lookahead_dist) {
        const double dx = refer_path[target_index + 1][0] - refer_path[target_index][0];
        const double dy = refer_path[target_index + 1][1] - refer_path[target_index][1];
        accumulated_dist += std::hypot(dx, dy);
        target_index++;
    }

    // 3. 坐标系转换（关键修正！）
    const double target_x_global = refer_path[target_index][0];
    const double target_y_global = refer_path[target_index][1];
    std::cout << "current_speed: " << current_speed << std::endl;
    std::cout << "current_x: " << current_x << std::endl;
    std::cout << "current_y: " << current_y << std::endl;
    std::cout << "current_yaw: " << current_yaw << std::endl;
    std::cout << "target_x_global: " << target_x_global << std::endl;
    std::cout << "target_y_global: " << target_y_global << std::endl;

    // 全局坐标 -> 车辆坐标系（Y轴向前，X轴向右）
    const double dx_global = target_x_global - current_x;  // 全局X差（车辆右侧）
    const double dy_global = target_y_global - current_y;  // 全局Y差（车辆前方）

    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);

    // 转换到车辆坐标系（注意符号）
    const double dx_vehicle = dx_global * cos_yaw - dy_global * sin_yaw;  // 车辆右侧为正
    const double dy_vehicle = dx_global * sin_yaw + dy_global * cos_yaw;  // 车辆前方为正

    // 横向误差 = 目标点在车辆坐标系中的X坐标（右侧为正）
    const double cte = dx_vehicle;

    // 4. PID转向控制（修正转向方向）
    const double dt = 0.1;
    static double prev_cte = 0.0;
    const double deriv_cte = (cte - prev_cte) / dt;
    prev_cte = cte;

    // 转向公式：正CTE（目标在右侧）需左转（负角度）
    double steer_angle = -(kp_steer * cte + kd_steer * deriv_cte);
    steer_angle = std::clamp(steer_angle, -max_steer_angle, max_steer_angle);

    // 5. 速度控制（保持简单）
    const double speed_error = target_speed - current_speed;
    output.accel_pct = (speed_error > 0) ? std::clamp(speed_error * 0.5, 0.0, 1.0) : 0.0;
    output.brake_pct = (speed_error < 0) ? std::clamp(-speed_error * 0.5, 0.0, 1.0) : 0.0;

    // 终点处理
    const double dist_to_end = std::hypot(refer_path.back()[0] - current_x, refer_path.back()[1] - current_y);
    if (dist_to_end < 2.0) {
        steer_angle *= 0.3;  // 终点降低转向灵敏度
        output.accel_pct *= 0.5;
    }

    output.steer_angle = steer_angle;

    // 调试输出
    ROS_INFO_THROTTLE(0.5, "CTE: %.2f | Steer: %.2f° | Accel: %.2f | Brake: %.2f | DY_global: %.2f", cte,
                      steer_angle * 180 / M_PI, output.accel_pct, output.brake_pct, dy_global);

    return output;
}

// 辅助函数：数值限幅
double clamp(double value, double min_val, double max_val) { return std::max(min_val, std::min(value, max_val)); }

double Controller::PIDController(const vector<double>& robot_state, const vector<vector<double>>& refer_path) {
    double Kp = 0.5;
    double Ki = 0;
    double Kd = 10;
    static double sum_error = 0;
    static double pre_error = 0;

    double min_ind = calTargetIndex(robot_state, refer_path);
    double alpha = atan2(refer_path[min_ind][1] - robot_state[1], refer_path[min_ind][0] - robot_state[0]);
    double l_d =
        sqrt(pow(refer_path[min_ind][0] - robot_state[0], 2) + pow(refer_path[min_ind][1] - robot_state[1], 2));
    double theta_e = alpha - robot_state[2];
    double e_y = -l_d * sin(theta_e);

    double error = 0 - e_y;
    double u = error * Kp + sum_error * Ki + (error - pre_error) * Kd;
    u = u > PI / 6 ? PI / 6 : u;
    u = u < -PI / 6 ? -PI / 6 : u;
    pre_error = error;
    sum_error += error;

    return u;
}

double Controller::PurePursuitController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                         const KinematicModel& ugv) {
    // Your code
    double l_d = 3;  // 前视距离
    /*---------- 1. 提取机器人状态 ----------*/
    double x = robot_state[0];      // 当前x坐标
    double y = robot_state[1];      // 当前y坐标
    double theta = robot_state[2];  // 当前航向角（弧度）

    /*---------- 2. 确定前视目标点 ----------*/
    int target_index = calTargetIndex(robot_state, refer_path);
    double current_dist = 0.0;

    // 从最近点开始向前搜索满足前视距离的点
    for (; target_index < refer_path.size(); ++target_index) {
        double dx = refer_path[target_index][0] - x;
        double dy = refer_path[target_index][1] - y;
        current_dist = sqrt(dx * dx + dy * dy);

        if (current_dist >= l_d) break;
    }

    // 边界检查：如果路径终点仍不满足前视距离
    if (target_index >= refer_path.size()) {
        target_index = refer_path.size() - 1;
    }

    /*---------- 3. 坐标变换到车辆坐标系 ----------*/
    double target_x = refer_path[target_index][0];
    double target_y = refer_path[target_index][1];

    // 计算全局坐标系下的差值
    double dx = target_x - x;
    double dy = target_y - y;

    // 进行坐标旋转（转换到车辆坐标系）
    double x_vehicle = dx * cos(theta) + dy * sin(theta);
    double y_vehicle = -dx * sin(theta) + dy * cos(theta);

    // 计算实际前视距离
    double d = sqrt(dx * dx + dy * dy);

    /*---------- 4. 计算转向曲率 ----------*/
    double kappa = 0.0;
    if (abs(d) > 1e-6) {                  // 避免除以零
        kappa = 2 * y_vehicle / (d * d);  // 纯追踪核心公式
    }

    /*---------- 5. 计算转向角 ----------*/
    double delta = atan(kappa * ugv.L);  // 自行车模型转换

    return delta;
}

double Controller::StanlyController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                    const KinematicModel& ugv) {
    // 1. 获取车辆当前状态
    double x = robot_state[0];    // x 坐标
    double y = robot_state[1];    // y 坐标
    double psi = robot_state[2];  // 航向角
    double v = robot_state[3];    // 速度

    // 2. 找到参考路径上的最近点
    int target_index = calTargetIndex(robot_state, refer_path);
    double ref_x = refer_path[target_index][0];
    double ref_y = refer_path[target_index][1];
    double ref_yaw = refer_path[target_index][2];

    // 3. 计算横向误差 e_y
    double dx = ref_x - x;
    double dy = ref_y - y;
    double e_y = dy * cos(ref_yaw) - dx * sin(ref_yaw);

    // 4. 计算航向误差 theta_e
    double theta_e = normalizeAngle(ref_yaw - psi);

    // 5. 计算控制量 (Stanley 控制公式)
    double k = 2;           // 控制增益（可以调节）
    double epsilon = 1e-6;  // 防止除零
    double delta = theta_e + atan2(k * e_y, v + epsilon);

    // 6. 限制转向角范围
    double max_steer = PI / 3;  // 最大转向角 60 度
    if (delta > max_steer) delta = max_steer;
    if (delta < -max_steer) delta = -max_steer;

    return delta;
}

double Controller::LQRController(const vector<double>& robot_state, const vector<vector<double>>& refer_path,
                                 const KinematicModel& ugv) {
    // 1. 确定目标点
    int target_index = calTargetIndex(robot_state, refer_path);
    vector<double> ref_state = refer_path[target_index];
    double ref_x = ref_state[0];
    double ref_y = ref_state[1];
    double ref_yaw = ref_state[2];
    double ref_delta = 0.0;  // 假设参考控制输入为0（直线跟踪）

    // 2. 计算状态误差（横向误差、航向误差）
    double dx = robot_state[0] - ref_x;
    double dy = robot_state[1] - ref_y;
    double e_psi = normalizeAngle(robot_state[2] - ref_yaw);  // 航向误差

    // 3. 获取线性化模型
    vector<MatrixXd> AB = ugv.stateSpace(ref_delta, ref_yaw);
    MatrixXd A = AB[0];  // 3x3
    MatrixXd B = AB[1];  // 3x2

    // 4. 设置权重矩阵（修正R的维度）
    MatrixXd Q = MatrixXd::Identity(3, 3);
    Q(0, 0) = 1.0;  // 横向误差
    Q(1, 1) = 1.0;  // 航向误差
    Q(2, 2) = 1.0;  // 纵向误差（未使用但保留维度）

    MatrixXd R = MatrixXd::Identity(2, 2) * 1;  // 控制量权重（2x2）

    // 5. 求解Riccati方程
    MatrixXd P = CalRicatti(A, B, Q, R);

    // 6. 计算反馈增益K（维度修正）
    MatrixXd K = (B.transpose() * P * B + R).inverse() * B.transpose() * P * A;

    // 7. 构建误差向量
    Vector3d error_state(dx, dy, e_psi);  // 第三维占位

    // 8. 计算控制量（提取转向角控制量）
    Vector2d u = -K * error_state;
    double delta = u(1);  // B矩阵第二列对应转向角

    // 9. 转向角限幅
    double max_delta = PI / 4;
    delta = std::max(std::min(delta, max_delta), -max_delta);

    return delta;
}