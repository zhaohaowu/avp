#include "planner.h"
#include <unsupported/Eigen/Splines>

Planner::Planner() {

}


vector<Vector2i> Planner::Bfs(const Vector2i &start, const Vector2i &goal,
                     const unordered_set<Vector2i, Vector2iHash>& obstacles) {
     // 如果起点或终点在障碍物中，直接返回空路径
    if (obstacles.count(start) || obstacles.count(goal)) {
        return {};
    }

    // 起点即终点的情况
    if (start == goal) {
        return {start};
    }

    std::queue<Vector2i> open_table;
    std::unordered_map<Vector2i, Vector2i, Vector2iHash> parent;
    const std::vector<Vector2i> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};

    open_table.push(start);
    parent[start] = Vector2i(-1, -1); // 特殊标记表示起点

    while (!open_table.empty()) {
        Vector2i current = open_table.front();
        open_table.pop();

        for (const auto& dir : directions) {
            Vector2i neighbor = current + dir;

            // 找到目标节点
            if (neighbor == goal) {
                parent[neighbor] = current;
                std::vector<Vector2i> path;
                Vector2i node = neighbor;
                
                // 回溯路径
                while (node.x() != -1 || node.y() != -1) {
                    path.push_back(node);
                    node = parent[node];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            // 检查邻居是否有效
            if (!obstacles.count(neighbor) && !parent.count(neighbor)) {
                parent[neighbor] = current;
                open_table.push(neighbor);
            }
        }
    }

    // 未找到路径
    return vector<Vector2i>();
}

vector<Vector2i> Planner::AStar(const Vector2i &start, const Vector2i &goal,
                                const unordered_set<Vector2i, Vector2iHash>& obstacles) {
    // 预处理检查
    if (obstacles.count(start) || obstacles.count(goal)) return {};
    if (start == goal) return {start};

    // 初始化数据结构
    std::unordered_map<Vector2i, int, Vector2iHash> g_score;  // 实际代价
    std::unordered_map<Vector2i, int, Vector2iHash> f_score;   // 预估总代价
    std::unordered_map<Vector2i, Vector2i, Vector2iHash> came_from;
    const std::vector<Vector2i> directions = {{0,1}, {0,-1}, {1,0}, {-1,0}};

    // 自定义优先队列
    struct Vector2iCompare {
        // 用于优先队列的比较器
        std::unordered_map<Vector2i, int, Vector2iHash>& f_score_ref;
    
        explicit Vector2iCompare(std::unordered_map<Vector2i, int, Vector2iHash>& scores)
            : f_score_ref(scores) {}
    
        bool operator()(const Vector2i& a, const Vector2i& b) const {
            return f_score_ref[a] > f_score_ref[b];  // 小顶堆
        }
    };

    using OpenList = std::priority_queue<
        Vector2i,
        std::vector<Vector2i>,
        Vector2iCompare>;
    
    OpenList open_queue((Vector2iCompare(f_score)));
    auto heuristic = [](const Vector2i& a, const Vector2i& b) {
        return std::abs(a.x() - b.x()) + std::abs(a.y() - b.y());  // 曼哈顿距离
    };

    // 初始化起点
    g_score[start] = 0;
    f_score[start] = heuristic(start, goal);
    open_queue.push(start);
    came_from[start] = Vector2i(-1, -1);  // 特殊起始标记

    while (!open_queue.empty()) {
        Vector2i current = open_queue.top();
        open_queue.pop();

        // 提前终止条件
        if (current == goal) {
            std::vector<Vector2i> path;
            Vector2i node = current;
            while (node.x() != -1 || node.y() != -1) {  // 假设x,y是公共成员变量
                path.push_back(node);
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());
            vector<Vector2i> smoothed;
            if(path.empty()) return smoothed;
        
            Vector2i last_valid = path[0];
            smoothed.push_back(last_valid);
            
            // 射线投射法简化路径
            for(size_t i = 1; i < path.size(); ++i) {
                auto LineOfSight = [](const Vector2i& start, const Vector2i& end, 
                    const unordered_set<Vector2i, Vector2iHash>& obstacles) {
                    // Bresenham算法参数计算
                    int x0 = start.x();
                    int y0 = start.y();
                    int x1 = end.x();
                    int y1 = end.y();
                    
                    int dx = abs(x1 - x0);
                    int dy = -abs(y1 - y0);
                    int sx = (x0 < x1) ? 1 : -1;
                    int sy = (y0 < y1) ? 1 : -1;
                    int err = dx + dy;
                
                    // 遍历直线路径
                    while(true) {
                        // 检查当前网格是否是障碍物
                        if(obstacles.count(Vector2i(x0, y0))) 
                            return false;
                
                        // 到达终点时停止
                        if(x0 == x1 && y0 == y1) break;
                
                        int e2 = 2 * err;
                        if(e2 >= dy) {  // 水平步进
                            if(x0 == x1) break;
                            err += dy;
                            x0 += sx;
                        }
                        if(e2 <= dx) {  // 垂直步进
                            if(y0 == y1) break;
                            err += dx;
                            y0 += sy;
                        }
                    }
                    return true;
                };
                if(!LineOfSight(last_valid, path[i], obstacles)) {
                    last_valid = path[i-1];
                    smoothed.push_back(last_valid);
                }
            }
            smoothed.push_back(path.back());
            return smoothed;
        }

        // 扩展邻居节点
        for (const auto& dir : directions) {
            Vector2i neighbor = current + dir;
            
            // 障碍物检查
            if (obstacles.count(neighbor)) continue;

            // 计算新代价
            int tentative_g = g_score[current] + 1;

            // 更新更优路径
            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal);
                
                // 防止重复添加（优化性能）
                bool in_queue = false;
                std::vector<Vector2i> temp;  // 临时存储
                while (!open_queue.empty()) {
                    Vector2i tmp = open_queue.top();
                    open_queue.pop();
                    if (tmp == neighbor) in_queue = true;
                    temp.push_back(tmp);
                }
                for (auto& n : temp) open_queue.push(n);
                
                if (!in_queue) open_queue.push(neighbor);
            }
        }
    }
    return vector<Vector2i>();
}

struct Vector3iHash {
    size_t operator()(const Vector3i& v) const {
        size_t h1 = hash<int>{}(v.x());
        size_t h2 = hash<int>{}(v.y());
        size_t h3 = hash<int>{}(v.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

Vector3i discretizeState(const State& s, double xy_res, double theta_res) {
    int x = static_cast<int>(round(s.x / xy_res));
    int y = static_cast<int>(round(s.y / xy_res));
    double theta = fmod(s.theta, 2 * M_PI);
    if (theta < 0) theta += 2 * M_PI;
    int t = static_cast<int>(round(theta / theta_res));
    return Vector3i(x, y, t);
}

double heuristic(const State& current, const State& goal) {
    double dx = goal.x - current.x;
    double dy = goal.y - current.y;
    double distance = hypot(dx, dy);
    double dtheta = abs(goal.theta - current.theta);
    dtheta = fmod(dtheta, 2 * M_PI);
    if (dtheta > M_PI) dtheta = 2 * M_PI - dtheta;
    return distance + 0.5 * dtheta;
}

State* moveCar(const State& current, double delta, double direction, double wheelbase, double step_size) {
    if (abs(delta) < 1e-5) {
        double dist = step_size * direction;
        double x_new = current.x + dist * cos(current.theta);
        double y_new = current.y + dist * sin(current.theta);
        return new State(x_new, y_new, current.theta, 0, 0, nullptr);
    } else {
        double R = wheelbase / tan(delta);
        double gamma = (step_size * direction) / R;
        double theta_new = current.theta + gamma;
        double dx = R * (sin(theta_new) - sin(current.theta));
        double dy = R * (cos(current.theta) - cos(theta_new));
        double x_new = current.x + dx;
        double y_new = current.y + dy;
        return new State(x_new, y_new, theta_new, 0, 0, nullptr);
    }
}

bool isGoal(const State& current, const State& goal, double xy_tol = 0.5, double theta_tol = M_PI/6) {
    double dx = current.x - goal.x;
    double dy = current.y - goal.y;
    double dtheta = abs(current.theta - goal.theta);
    dtheta = fmod(dtheta, 2*M_PI);
    if (dtheta > M_PI) dtheta = 2*M_PI - dtheta;
    return (hypot(dx, dy) <= xy_tol && dtheta <= theta_tol);
}

std::vector<State*> smoothPath(const std::vector<State*>& path) {
    if (path.size() < 3) {
        return path;
    }

    Eigen::MatrixXd points(2, path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        points(0, i) = path[i]->x;
        points(1, i) = path[i]->y;
    }

    Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, 3);
    
    std::vector<State*> smoothed;
    for (double t = 0; t <= 1.0; t += 0.02) {
        Eigen::Vector2d pt = spline(t);
        smoothed.emplace_back(new State(pt(0), pt(1), 0, 0, 0, nullptr));
    }
    return smoothed;
}

std::vector<State*> Planner::HybridAStar(const State& start, const State& goal,
                                        const std::unordered_set<Vector2i, Vector2iHash>& obstacles_index,
                                        double wheelbase, double step_size, double max_steer) {
    const double xy_res = 0.5;
    const double theta_res = M_PI / 18;
    
    auto cmp = [](State* a, State* b) { return a->f > b->f; };
    std::priority_queue<State*, std::vector<State*>, decltype(cmp)> open_queue(cmp);
    std::unordered_map<Vector3i, double, Vector3iHash> visited;

    State* start_state = new State(start);
    start_state->f = heuristic(start, goal);
    open_queue.push(start_state);
    visited[discretizeState(*start_state, xy_res, theta_res)] = 0;

    while (!open_queue.empty()) {
        State* current = open_queue.top();
        open_queue.pop();

        if (isGoal(*current, goal)) {
            std::vector<State*> path;
            for (State* node = current; node; node = node->parent)
                path.push_back(node);
            std::reverse(path.begin(), path.end());
            
            // Cleanup unused nodes
            while (!open_queue.empty()) {
                delete open_queue.top();
                open_queue.pop();
            }
            auto smoothed = smoothPath(path);
            return smoothed;
        }

        for (double delta : {max_steer, 0.0, -max_steer}) {
            for (double dir : {1.0, -1.0}) {
                State* new_state = moveCar(*current, delta, dir, wheelbase, step_size);
                Vector2i grid(round(new_state->x / xy_res), round(new_state->y / xy_res));
                
                if (obstacles_index.count(grid)) {
                    delete new_state;
                    continue;
                }

                Vector3i disc = discretizeState(*new_state, xy_res, theta_res);
                new_state->g = current->g + step_size;
                new_state->f = new_state->g + heuristic(*new_state, goal);
                new_state->parent = current;

                auto it = visited.find(disc);
                if (it != visited.end() && new_state->g >= it->second) {
                    delete new_state;
                    continue;
                }

                visited[disc] = new_state->g;
                open_queue.push(new_state);
            }
        }
    }
    return {};
}