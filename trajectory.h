#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>

#include "sim_types.h"

std::vector<RefPoint> BuildArcTrajectory();
RefPoint InterpolateByTime(const std::vector<RefPoint>& traj, double t);
RefPoint InterpolateByS(const std::vector<RefPoint>& traj, double s);
FrenetMatch MatchToTrajectory(const std::vector<RefPoint>& traj,
                              const VehicleState& st,
                              std::size_t hint_idx);

#endif
