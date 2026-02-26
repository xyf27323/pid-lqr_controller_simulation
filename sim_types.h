#ifndef SIM_TYPES_H_
#define SIM_TYPES_H_

#include <algorithm>
#include <cmath>
#include <cstddef>

constexpr double kPi = 3.14159265358979323846;

// 参考轨迹离散点。
struct RefPoint {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double t = 0.0;
};

// 车辆状态（以后轴中心为参考点）。
struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double omega = 0.0;
  double steer = 0.0;
  double v = 0.0;
  double a = 0.0;
  double s = 0.0;
};

// 轨迹匹配结果：离散索引 + 连续 Frenet 坐标。
struct FrenetMatch {
  std::size_t idx = 0;
  double s = 0.0;
  double d = 0.0;
  RefPoint ref;
};

inline double Clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

inline double NormalizeAngle(double a) {
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

#endif
