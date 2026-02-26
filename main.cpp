#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "lqr_controller.h"
#include "matplotlibcpp.h"
#include "pid_controller.h"
#include "trajectory.h"
#include "visualizer.h"

namespace plt = matplotlibcpp;

int main() {
  std::filesystem::remove_all("output/frames");
  std::filesystem::create_directories("output/frames");
  std::filesystem::remove("output/demo.gif");

  // 1) 构造参考轨迹。
  const auto ref = BuildArcTrajectory();

  // 2) 纵向双环 PID 参数。
  PidConfig station_pid_conf;
  station_pid_conf.integrator_enable = false;
  station_pid_conf.integrator_saturation_level = 0.1;
  station_pid_conf.output_saturation_level = 3.0;
  station_pid_conf.kp = 15.0;
  station_pid_conf.ki = 0.2;
  station_pid_conf.kd = 0.01;

  PidConfig speed_pid_conf;
  speed_pid_conf.integrator_enable = true;
  speed_pid_conf.integrator_saturation_level = 0.3;
  speed_pid_conf.output_saturation_level = 3.0;
  speed_pid_conf.kp = 1.5;
  speed_pid_conf.ki = 0.5;
  speed_pid_conf.kd = 0.0;
  speed_pid_conf.kaw = 1.0;

  PIDController station_pid;
  PIDController speed_pid;
  station_pid.Init(station_pid_conf);
  speed_pid.Init(speed_pid_conf);

  const double ts = 0.01;
  const double wheelbase = 2.72;
  LqrController lqr;

  // 3) 初始状态：放在起点法向外侧，带一点航向偏差。
  VehicleState st;
  const RefPoint& p0 = ref.front();
  const double n_x = -std::sin(p0.theta);
  const double n_y = std::cos(p0.theta);
  st.x = p0.x + 3.0 * n_x;
  st.y = p0.y + 3.0 * n_y;
  st.theta = NormalizeAngle(p0.theta - 0.12);

  std::size_t hint_idx = 0;
  std::vector<double> traj_x;
  std::vector<double> traj_y;
  traj_x.reserve(5000);
  traj_y.reserve(5000);

  std::vector<double> ref_x;
  std::vector<double> ref_y;
  ref_x.reserve(ref.size());
  ref_y.reserve(ref.size());
  for (const auto& p : ref) {
    ref_x.push_back(p.x);
    ref_y.push_back(p.y);
  }

  const double total_time = ref.back().t + 6.0;
  const int max_steps = static_cast<int>(total_time / ts);
  const int frame_stride = 10;

  PlotConfig plot_cfg;

  const bool headless = (std::getenv("SIM_HEADLESS") != nullptr);
  if (headless) {
    plt::backend("Agg");
  }
  plt::figure_size(1200, 900);
  if (!headless) {
    plt::show(false);
  }

  int frame_id = 0;
  for (int k = 0; k < max_steps; ++k) {
    // 4) 匹配当前车辆到参考轨迹，提取横向/纵向控制目标。
    const FrenetMatch match = MatchToTrajectory(ref, st, hint_idx);
    const RefPoint longitudinal_target = match.ref;
    hint_idx = match.idx;
    st.s = match.s;

    const double remaining_s = std::max(0.0, ref.back().s - st.s);
    const double end_dx = st.x - ref.back().x;
    const double end_dy = st.y - ref.back().y;
    const double end_dist = std::hypot(end_dx, end_dy);

    const double station_error = longitudinal_target.s - st.s;

    // 5) 纵向参考速度限制：几何收敛约束 + 制动距离约束。
    double v_ref = longitudinal_target.v;
    const double v_brake_limit = std::sqrt(std::max(0.0, 2.0 * 1.8 * remaining_s));
    const double v_geo_limit = 2.5 * end_dist;
    v_ref = std::min(v_ref, std::min(v_brake_limit, v_geo_limit));
    if (remaining_s <= 1.0) {
      v_ref = 0.0;
    }

    const double speed_error = v_ref - st.v;
    const double speed_offset = station_pid.Control(station_error, ts);
    const double speed_input = Clamp(speed_offset + speed_error, -3.0, 3.0);
    double a_cmd = speed_pid.Control(speed_input, ts) + longitudinal_target.a;

    // 超速时附加制动，避免终点附近低速拖尾。
    if (st.v > v_ref + 0.15) {
      a_cmd = std::min(a_cmd, -(st.v - v_ref));
    }

    if (end_dist < 1.2) {
      a_cmd = std::min(a_cmd, 0.0);
      if (st.v > 0.02) {
        a_cmd = std::min(a_cmd, -2.0);
      }
    }

    a_cmd = Clamp(a_cmd, -4.0, 2.5);

    // 6) 横向 LQR 转角控制。
    LqrDebug lqr_debug;
    const double delta_cmd = lqr.ComputeSteering(st, match, &lqr_debug);

    // 7) 车辆离散更新（后轴模型）。
    st.steer = delta_cmd;
    st.a = a_cmd;
    st.v = std::max(0.0, st.v + st.a * ts);
    st.omega = st.v / wheelbase * std::tan(st.steer);
    st.theta = NormalizeAngle(st.theta + st.omega * ts);
    st.x += st.v * std::cos(st.theta) * ts;
    st.y += st.v * std::sin(st.theta) * ts;

    // 8) 终点停车判据。
    const FrenetMatch stop_match = MatchToTrajectory(ref, st, hint_idx);
    const double remaining_s_now = std::max(0.0, ref.back().s - stop_match.s);

    bool stop_now = false;
    if ((end_dist < 0.12 && st.v < 0.08) ||
        (remaining_s_now <= 1.0 && st.v < 0.03)) {
      st.v = 0.0;
      st.a = 0.0;
      a_cmd = 0.0;
      stop_now = true;
    }

    traj_x.push_back(st.x);
    traj_y.push_back(st.y);

    if (k % frame_stride == 0 || stop_now) {
      std::ostringstream filename;
      filename << "output/frames/frame_" << std::setfill('0') << std::setw(4)
               << frame_id++ << ".png";

      RenderFrame(plot_cfg, ref_x, ref_y, traj_x, traj_y, st, match,
                  longitudinal_target, lqr_debug, v_ref, a_cmd, wheelbase,
                  filename.str());
      if (!headless) {
        plt::pause(0.001);
      }
    }

    if (stop_now) {
      break;
    }
  }

  if (!headless) {
    std::cout << "Simulation reached final frame. Window will stay for 5 seconds...\n";
    plt::pause(5.0);
  }

  const int gif_ret =
      std::system("python3 scripts/make_gif.py output/frames output/demo.gif 12");
  if (gif_ret != 0) {
    std::cerr << "Failed to generate gif automatically. Run: "
              << "python3 scripts/make_gif.py output/frames output/demo.gif 12\n";
  }

  std::cout << "Simulation finished. GIF: output/demo.gif\n";
  std::cout << "final s=" << st.s << ", final v=" << st.v
            << ", ref_end_s=" << ref.back().s << "\n";
  return 0;
}
