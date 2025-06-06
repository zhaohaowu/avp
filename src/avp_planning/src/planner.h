#ifndef AVP_WS_PLANNER_H
#define AVP_WS_PLANNER_H

#endif //AVP_WS_PLANNER_H

#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <algorithm>  // reverse
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

struct Vector2iHash {
    size_t operator()(const Vector2i &v) const {
        size_t h1 = hash<int>{}(v.x());
        size_t h2 = hash<int>{}(v.y());
        return h1 ^ (h2 << 1);
    }
};

// 定义车辆状态
struct State {
    double x, y, theta; // 位置和朝向角
    double g;           // 累计代价
    double f;           // 总估计代价
    State* parent;      // 父节点指针

    State(double x, double y, double theta, double g, double f, State* parent)
            : x(x), y(y), theta(theta), g(g), f(f), parent(parent) {}

    // 比较器，用于优先队列
    bool operator>(const State& other) const {
        return f > other.f;
    }
};

class Planner {
public:
    Planner();
    vector<Vector2i> Bfs(const Vector2i &start, const Vector2i &goal,
                                  const unordered_set<Vector2i, Vector2iHash>& obstacles);
    vector<Vector2i> AStar(const Vector2i &start, const Vector2i &goal,
                                    const unordered_set<Vector2i, Vector2iHash>& obstacles);

    std::vector<State*> HybridAStar(const State& start, const State& goal,
                                    const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
                                    double wheelbase, double step_size, double max_steer);
};