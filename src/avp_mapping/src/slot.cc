#include "slot.h"

#include <cmath>


namespace {
double computeCenterDistance(cv::Vec4i line1, cv::Vec4i line2) {
    cv::Point2f p1((line1[0] + line1[2]) / 2.0f, (line1[1] + line1[3]) / 2.0f);
    cv::Point2f p2((line2[0] + line2[2]) / 2.0f, (line2[1] + line2[3]) / 2.0f);
    return cv::norm(p1 - p2);
}

cv::Point2f computeIntersect(cv::Vec4i line1, cv::Vec4i line2) {
    int x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
    int x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];
    float denom = (float)((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    cv::Point2f intersect((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4),
                          (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4));
    return intersect / denom;
}
}  // end of namespace

// 保留边角、细小区域 和骨架
cv::Mat skeletonize(const cv::Mat& img) {
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat temp, eroded;
    do {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(img, temp, temp);  // 细小区域
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);
    } while (0 != cv::countNonZero(img));
    return skel;
}

void removeIsolatedPixels(cv::Mat& src, int min_neighbors) {
    CV_Assert(src.type() == CV_8UC1);  // 确保单通道灰度图
    cv::Mat dst = src.clone();
    for (int y = 1; y < src.rows - 1; ++y) {
        for (int x = 1; x < src.cols - 1; ++x) {
            if (src.at<uchar>(y, x) > 0) {  // 如果是前景图像
                int neighbor_count = 0;     // 相邻前景像素的计数器
                // 检查8邻域内的像素
                for (int ny = -1; ny <= 1; ++ny) {
                    for (int nx = -1; nx <= 1; ++nx) {
                        if (ny == 0 && nx == 0) continue;  // 跳过自己
                        if (src.at<uchar>(y + ny, x + nx) > 0) {
                            ++neighbor_count;  // 增加邻居数
                        }
                    }
                }
                // 如果相邻前景像素小于阈值，则认为该点是孤立的，并将其去除
                if (neighbor_count < min_neighbors) {
                    dst.at<uchar>(y, x) = 0;
                }
            }
        }
    }
    src = dst;
}

bool isSlotShortLine(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Mat& image) {
    cv::LineIterator it(image, point1, point2, 4);
    int positiveIndex = 0;
    const double len = cv::norm(point1 - point2);
    int delta = 10;
    if (std::fabs(len - kShortLinePixelDistance) < delta) {
        for (int i = 0; i < it.count; ++i, ++it) {
            int color = image.at<uchar>(std::round(it.pos().y), std::round(it.pos().x));
            if (color > 0) {
                positiveIndex++;
            }
        }
        if (positiveIndex > kShortLineDetectPixelNumberMin) {
            return true;
        }
    }
    return false;
}

bool isSlotLongLine(const cv::Mat& line_img, const cv::Point2f& start, const cv::Point2f& dir) {
    int cnt{0};
    cv::Point2f pt(start);
    for (int l = 1; l < kLongLinePixelDistance; ++l) {
        pt += dir;
        if (pt.y <= 0 || pt.y >= kIPMImgHeight || pt.x <= 0 || pt.x >= kIPMImgWidth) {
            continue;
        }
        if (line_img.at<uchar>(pt) > 0) {
            ++cnt;
        }
    }
    return cnt > kLongLineDetectPixelNumberMin;
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> detectSlot(const cv::Mat& slot_img,
                                                                          const std::vector<cv::Vec4i>& lines) {
    const float kSlotShortLength = 4.2;   // 车位短边长度（米）
    const float kSlotLongLength = 6.4;    // 车长长度（米）
    const float kPixelResolution = 0.02;  // 米/像素
    const float kAngleThreshold = 5.0;    // 角度容差
    const int kLineCheckSteps = 20;       // 线段采样检查步数

    /////////////// TODO 1 detect corners from lines ///////////////
    std::vector<cv::Point2f> corners;  // corners detected from lines
    for (size_t i = 0; i < lines.size(); ++i) {
        const auto& line1 = lines[i];
        cv::Point2f p1(line1[0], line1[1]);
        cv::Point2f p2(line1[2], line1[3]);

        for (size_t j = i + 1; j < lines.size(); ++j) {
            const auto& line2 = lines[j];
            cv::Point2f p3(line2[0], line2[1]);
            cv::Point2f p4(line2[2], line2[3]);
            // 去除重复的线
            if (cv::norm(p1 - p3) < 10 || cv::norm(p2 - p4) < 10) {
                std::cout << "重复的线" << std::endl;
                continue;
            }

            // 计算线段夹角
            Eigen::Vector2d vec1{p2.x - p1.x, p2.y - p1.y};
            Eigen::Vector2d vec2{p4.x - p3.x, p4.y - p3.y};
            float angle = std::acos(vec1.dot(vec2) / vec1.norm() * vec2.norm()) / M_PI * 180;
            if (std::abs(angle - 90) > kAngleThreshold) {
                std::cout << "angle " << angle << std::endl;
                continue;
            }

            // 计算直线交点
            cv::Point2f intersect;
            auto computeIntersection = [](cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4,
                                          cv::Point2f& intersect) {
                constexpr float epsilon = 1e-6f;  // 浮点精度阈值
                // 计算方向向量
                cv::Point2f dir1 = p2 - p1;
                cv::Point2f dir2 = p4 - p3;
                // 计算分母（向量叉积）
                float denom = dir1.x * dir2.y - dir1.y * dir2.x;
                // 处理平行直线
                if (std::fabs(denom) < epsilon) {
                    // 可选：处理重合直线的情况
                    return false;
                }
                // 计算分子项
                float s_nom = dir1.x * (p1.y - p3.y) - dir1.y * (p1.x - p3.x);
                float t_nom = dir2.x * (p1.y - p3.y) - dir2.y * (p1.x - p3.x);
                // 计算参数
                float s = s_nom / denom;
                float t = t_nom / denom;
                // 计算交点坐标（使用参数t代入第一条直线方程）
                intersect.x = p1.x + t * dir1.x;
                intersect.y = p1.y + t * dir1.y;
                return true;
            };
            if (computeIntersection(p1, p2, p3, p4, intersect)) {
                // 验证交点区域是否属于库位线
                if (intersect.x >= 0 && intersect.x < slot_img.cols && intersect.y >= 0 &&
                    intersect.y < slot_img.rows) {
                    if (slot_img.at<uchar>(intersect.y, intersect.x) == 254) {
                        corners.emplace_back(intersect);
                    }
                }
            }
        }
    }

    /////////////// TODO 2 detect slots in slot_img ///////////////
    std::vector<cv::Point2f> slot_points;

    // 验证短边特征
    std::vector<std::pair<cv::Point2f, cv::Point2f>> valid_edges;
    for (size_t i = 0; i < corners.size(); ++i) {
        for (size_t j = i + 1; j < corners.size(); ++j) {
            cv::Point2f pt1 = corners[i];
            cv::Point2f pt2 = corners[j];

            // 转换为物理距离
            float dist_pixels = cv::norm(pt1 - pt2);
            float dist_meters = dist_pixels * kPixelResolution;

            // 验证短边长度
            if (std::abs(dist_meters - kSlotShortLength) > 0.3) continue;

            // 验证线段上的点是否属于库位线
            int valid_count = 0;
            for (int k = 0; k <= kLineCheckSteps; ++k) {
                float ratio = static_cast<float>(k) / kLineCheckSteps;
                cv::Point2f sample_pt = pt1 * (1 - ratio) + pt2 * ratio;
                if (slot_img.at<uchar>(sample_pt.y, sample_pt.x) == 254) {
                    ++valid_count;
                }
            }
            if (valid_count < kLineCheckSteps * 0.5) continue;

            valid_edges.emplace_back(pt1, pt2);
        }
    }

    // 验证垂线特征并推测角点
    for (const auto& edge : valid_edges) {
        cv::Point2f dir = edge.second - edge.first;
        cv::Point2f perpendicular_dir1(dir.y, -dir.x);  // 右垂直方向（图像坐标系）
        cv::Point2f perpendicular_dir2(-dir.y, dir.x);  // 左垂直方向（图像坐标系）

        // 计算单位向量（考虑物理分辨率）
        auto normalize_to_resolution = [kPixelResolution](cv::Point2f v) {
            float length = cv::norm(v) * kPixelResolution;  // 物理长度（米）
            return (length > 1e-6) ? v * (1.0f / (length / kPixelResolution)) : cv::Point2f();
        };

        perpendicular_dir1 = normalize_to_resolution(perpendicular_dir1);
        perpendicular_dir2 = normalize_to_resolution(perpendicular_dir2);

        // 验证两个端点的垂线方向
        auto check_perpendicular = [&](cv::Point2f start, cv::Point2f dir) {
            const int required_steps = kSlotLongLength / kPixelResolution;
            int valid_count = 0;
            for (int k = 1; k <= required_steps; ++k) {
                cv::Point2f pt = start + dir * k;
                if (pt.x < 0 || pt.x >= slot_img.cols || pt.y < 0 || pt.y >= slot_img.rows) break;
                int x = cvRound(pt.x);
                int y = cvRound(pt.y);
                if (slot_img.at<uchar>(y, x) == 254) {
                    valid_count++;
                }
            }
            return valid_count > required_steps * 0.7;
        };

        // 双向验证
        bool valid_right =
            check_perpendicular(edge.first, perpendicular_dir1) && check_perpendicular(edge.second, perpendicular_dir1);

        bool valid_left =
            check_perpendicular(edge.first, perpendicular_dir2) && check_perpendicular(edge.second, perpendicular_dir2);

        if (valid_right || valid_left) {
            cv::Point2f base_dir = valid_right ? perpendicular_dir1 : perpendicular_dir2;

            // 保证四边形闭合
            cv::Point2f pt3 = edge.first + base_dir * (kSlotLongLength / kPixelResolution);
            cv::Point2f pt4 = edge.second + base_dir * (kSlotLongLength / kPixelResolution);

            // 按顺时针顺序存储四个角点
            std::vector<cv::Point2f> slot_corners{edge.first, edge.second, pt4, pt3};

            slot_points.insert(slot_points.end(), slot_corners.begin(), slot_corners.end());
        }
    }

    return std::make_tuple(corners, slot_points);
}
