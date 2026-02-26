#include "visualizer.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace plt = matplotlibcpp;

void PlotVehicleBox(const VehicleState& st, double wheelbase) {
  const double front = wheelbase + 1.0;
  const double rear = 1.2;
  const double half_w = 0.95;

  const std::vector<std::pair<double, double>> local = {
      {front, half_w},
      {front, -half_w},
      {-rear, -half_w},
      {-rear, half_w},
      {front, half_w},
  };

  std::vector<double> bx;
  std::vector<double> by;
  bx.reserve(local.size());
  by.reserve(local.size());

  const double c = std::cos(st.theta);
  const double s = std::sin(st.theta);
  for (const auto& p : local) {
    bx.push_back(st.x + c * p.first - s * p.second);
    by.push_back(st.y + s * p.first + c * p.second);
  }

  plt::plot(bx, by, {{"color", "red"}});
}

void RenderFrame(const PlotConfig& cfg,
                 const std::vector<double>& ref_x,
                 const std::vector<double>& ref_y,
                 const std::vector<double>& traj_x,
                 const std::vector<double>& traj_y,
                 const VehicleState& st,
                 const FrenetMatch& match,
                 const RefPoint& longitudinal_target,
                 const LqrDebug& lqr_debug,
                 double v_ref,
                 double a_cmd,
                 double wheelbase,
                 const std::string& out_file) {
  plt::clf();
  plt::xlim(cfg.x_min, cfg.x_max);
  plt::ylim(cfg.y_min, cfg.y_max);
  plt::grid(true);
  plt::axis("equal");

  plt::plot(ref_x, ref_y,
            {{"label", "Reference path"}, {"color", "black"}, {"linestyle", "--"}});
  plt::plot(traj_x, traj_y,
            {{"label", "Vehicle trajectory"}, {"color", "blue"}, {"linestyle", "-"}});

  PlotVehicleBox(st, wheelbase);

  plt::scatter(std::vector<double>{match.ref.x}, std::vector<double>{match.ref.y},
               60.0, {{"color", "orange"}, {"label", "Lateral target"}});
  plt::scatter(std::vector<double>{longitudinal_target.x}, std::vector<double>{longitudinal_target.y},
               60.0, {{"color", "magenta"}, {"label", "Longitudinal target"}});
  plt::scatter(std::vector<double>{st.x}, std::vector<double>{st.y},
               40.0, {{"color", "black"}, {"label", "Rear axle"}});

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4)
      << "LATERAL LQR (dynamic model)\n"
      << "Q=[0.01,0,0.05,0], R=0.05\n"
      << "e_y=" << lqr_debug.e_y << " m, e_psi=" << lqr_debug.e_psi << " rad\n"
      << "delta_fb=" << lqr_debug.delta_fb << " rad, delta_ff=" << lqr_debug.delta_ff
      << " rad\n\n"
      << "LONGITUDINAL PID (dual-loop)\n"
      << "Station PID: Kp=15 Ki=0.2 Kd=0.01\n"
      << "Speed PID: Kp=1.5 Ki=0.5 Kd=0.0\n"
      << "s=" << st.s << " m, v_ref=" << v_ref << " m/s, v_ego=" << st.v
      << " m/s, a_cmd=" << a_cmd;
  plt::text(cfg.text_x, cfg.text_y, oss.str());

  plt::legend("lower left");
  plt::save(out_file);
}
