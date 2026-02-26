#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <string>
#include <vector>

#include "lqr_controller.h"
#include "matplotlibcpp.h"
#include "sim_types.h"

struct PlotConfig {
  double x_min = -102.0;
  double x_max = 40.0;
  double y_min = -40.0;
  double y_max = 40.0;
  double text_x = -106.0;
  double text_y = 43.0;
};

void PlotVehicleBox(const VehicleState& st, double wheelbase);

void RenderFrame(const PlotConfig& cfg,
                 const std::vector<double>& ref_x,
                 const std::vector<double>& ref_y,
                 const std::vector<double>& traj_x,
                 const std::vector<double>& traj_y,
                 const VehicleState& st,
                 const FrenetMatch& match,
                 const RefPoint& target_t,
                 const LqrDebug& lqr_debug,
                 double v_ref,
                 double a_cmd,
                 double wheelbase,
                 const std::string& out_file);

#endif
