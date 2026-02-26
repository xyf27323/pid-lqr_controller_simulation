#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>

#include "sim_types.h"

// 生成参考圆弧轨迹（含 s/v/a/t 信息）。
std::vector<RefPoint> BuildArcTrajectory();

// 按弧长进行插值，返回连续参考点。
RefPoint InterpolateByS(const std::vector<RefPoint>& traj, double s);

// 将车辆状态匹配到轨迹，返回最近参考点及 Frenet 坐标。
FrenetMatch MatchToTrajectory(const std::vector<RefPoint>& traj,
                              const VehicleState& st,
                              std::size_t hint_idx);

#endif
