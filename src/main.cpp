#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "common/math/linear_quadratic_regulator.h"
#include "control/control/pid_controller.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace {

constexpr double kPi = 3.14159265358979323846;

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

struct FrenetMatch {
  size_t idx = 0;
  double s = 0.0;
  double d = 0.0;
  RefPoint ref;
};

double Clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

double NormalizeAngle(double a) {
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

std::vector<RefPoint> BuildArcTrajectory() {
  const double radius = 32.0;
  const double ds = 0.5;
  const double total_s = 200.0;
  const size_t n = static_cast<size_t>(total_s / ds) + 1;
  const double phi0 = -2.35;
  const double kappa = -1.0 / radius;
  const double v_cruise = 8.0;

  std::vector<RefPoint> traj;
  traj.reserve(n);
  double t = 0.0;

  for (size_t i = 0; i < n; ++i) {
    const double s = static_cast<double>(i) * ds;
    const double progress = s / total_s;
    const double phi = phi0 - s / radius;

    RefPoint p;
    p.x = radius * std::cos(phi);
    p.y = radius * std::sin(phi);
    p.theta = NormalizeAngle(phi - kPi / 2.0);
    p.kappa = kappa;
    p.s = s;

    double start_scale = 1.0;
    if (progress < 0.12) {
      start_scale = std::sin(0.5 * kPi * (progress / 0.12));
    }
    double end_scale = 1.0;
    if (progress > 0.72) {
      end_scale = std::cos(0.5 * kPi * ((progress - 0.72) / 0.28));
      end_scale = std::max(0.0, end_scale);
    }
    p.v = v_cruise * start_scale * end_scale;

    if (i > 0) {
      const double v_avg = std::max(0.2, 0.5 * (traj.back().v + p.v));
      t += ds / v_avg;
    }
    p.t = t;

    traj.push_back(p);
  }

  for (size_t i = 1; i + 1 < traj.size(); ++i) {
    const double dt = std::max(1e-3, traj[i + 1].t - traj[i - 1].t);
    traj[i].a = (traj[i + 1].v - traj[i - 1].v) / dt;
  }
  if (traj.size() >= 2) {
    traj.front().a = traj[1].a;
    traj.back().a = traj[traj.size() - 2].a;
  }

  return traj;
}

RefPoint InterpolateByTime(const std::vector<RefPoint>& traj, double t) {
  if (t <= traj.front().t) {
    return traj.front();
  }
  if (t >= traj.back().t) {
    return traj.back();
  }

  size_t lo = 0;
  size_t hi = traj.size() - 1;
  while (lo + 1 < hi) {
    const size_t mid = (lo + hi) / 2;
    if (traj[mid].t < t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const RefPoint& p0 = traj[lo];
  const RefPoint& p1 = traj[hi];
  const double r = Clamp((t - p0.t) / std::max(1e-6, p1.t - p0.t), 0.0, 1.0);

  RefPoint out;
  out.x = p0.x + (p1.x - p0.x) * r;
  out.y = p0.y + (p1.y - p0.y) * r;
  out.theta = NormalizeAngle(p0.theta + NormalizeAngle(p1.theta - p0.theta) * r);
  out.kappa = p0.kappa + (p1.kappa - p0.kappa) * r;
  out.s = p0.s + (p1.s - p0.s) * r;
  out.v = p0.v + (p1.v - p0.v) * r;
  out.a = p0.a + (p1.a - p0.a) * r;
  out.t = t;
  return out;
}

RefPoint InterpolateByS(const std::vector<RefPoint>& traj, double s) {
  if (s <= traj.front().s) {
    return traj.front();
  }
  if (s >= traj.back().s) {
    return traj.back();
  }

  size_t lo = 0;
  size_t hi = traj.size() - 1;
  while (lo + 1 < hi) {
    const size_t mid = (lo + hi) / 2;
    if (traj[mid].s < s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const RefPoint& p0 = traj[lo];
  const RefPoint& p1 = traj[hi];
  const double r = Clamp((s - p0.s) / std::max(1e-6, p1.s - p0.s), 0.0, 1.0);

  RefPoint out;
  out.x = p0.x + (p1.x - p0.x) * r;
  out.y = p0.y + (p1.y - p0.y) * r;
  out.theta = NormalizeAngle(p0.theta + NormalizeAngle(p1.theta - p0.theta) * r);
  out.kappa = p0.kappa + (p1.kappa - p0.kappa) * r;
  out.s = s;
  out.v = p0.v + (p1.v - p0.v) * r;
  out.a = p0.a + (p1.a - p0.a) * r;
  out.t = p0.t + (p1.t - p0.t) * r;
  return out;
}

FrenetMatch MatchToTrajectory(const std::vector<RefPoint>& traj,
                             const VehicleState& st,
                             size_t hint_idx) {
  const size_t n = traj.size();
  const size_t start = (hint_idx > 30) ? hint_idx - 30 : 0;
  const size_t end = std::min(n - 1, hint_idx + 50);

  size_t best = start;
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = start; i <= end; ++i) {
    const double dx = st.x - traj[i].x;
    const double dy = st.y - traj[i].y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best = i;
    }
  }

  const RefPoint& ref = traj[best];
  const double dx = st.x - ref.x;
  const double dy = st.y - ref.y;
  const double cos_t = std::cos(ref.theta);
  const double sin_t = std::sin(ref.theta);

  FrenetMatch m;
  m.idx = best;
  m.s = ref.s + dx * cos_t + dy * sin_t;
  m.d = cos_t * dy - sin_t * dx;
  m.ref = InterpolateByS(traj, Clamp(m.s, traj.front().s, traj.back().s));
  return m;
}

void PlotVehicleBox(const VehicleState& st, double wheelbase) {
  const double front = wheelbase + 1.0;
  const double rear = 1.2;
  const double half_w = 0.95;

  std::vector<std::pair<double, double>> local = {
      {front, half_w}, {front, -half_w}, {-rear, -half_w}, {-rear, half_w}, {front, half_w}};

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

}  // namespace

int main() {
  std::filesystem::create_directories("output/frames");

  const auto ref = BuildArcTrajectory();

  controller::PidConf station_pid_conf;
  station_pid_conf.set_integrator_enable(false);
  station_pid_conf.set_integrator_saturation_level(0.1);
  station_pid_conf.set_output_saturation_level(3.0);
  station_pid_conf.set_kp(15.0);
  station_pid_conf.set_ki(0.2);
  station_pid_conf.set_kd(0.01);

  controller::PidConf speed_pid_conf;
  speed_pid_conf.set_integrator_enable(true);
  speed_pid_conf.set_integrator_saturation_level(0.3);
  speed_pid_conf.set_output_saturation_level(3.0);
  speed_pid_conf.set_kp(1.5);
  speed_pid_conf.set_ki(0.5);
  speed_pid_conf.set_kd(0.0);
  speed_pid_conf.set_kaw(1.0);

  PIDController station_pid;
  PIDController speed_pid;
  station_pid.Init(station_pid_conf);
  speed_pid.Init(speed_pid_conf);

  const double ts = 0.01;
  const double wheelbase = 2.72;
  const double cf = 155494.663;
  const double cr = 155494.663;
  const double mass_fl = 520.0;
  const double mass_fr = 520.0;
  const double mass_rl = 520.0;
  const double mass_rr = 520.0;
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  const double mass = mass_front + mass_rear;
  const double lf = wheelbase * (1.0 - mass_front / mass);
  const double lr = wheelbase * (1.0 - mass_rear / mass);
  const double iz = lf * lf * mass_front + lr * lr * mass_rear;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd A_coeff = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 4);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  A(0, 1) = 1.0;
  A(1, 2) = (cf + cr) / mass;
  A(2, 3) = 1.0;
  A(3, 2) = (lf * cf - lr * cr) / iz;

  A_coeff(1, 1) = -(cf + cr) / mass;
  A_coeff(1, 3) = (lr * cr - lf * cf) / mass;
  A_coeff(3, 1) = (lr * cr - lf * cf) / iz;
  A_coeff(3, 3) = -(lf * lf * cf + lr * lr * cr) / iz;

  B(1, 0) = cf / mass;
  B(3, 0) = lf * cf / iz;
  Bd = B * ts;

  Q(0, 0) = 0.01;
  Q(1, 1) = 0.0;
  Q(2, 2) = 0.05;
  Q(3, 3) = 0.0;
  R(0, 0) = 0.05;

  VehicleState st;
  const RefPoint& p0 = ref.front();
  const double n_x = -std::sin(p0.theta);
  const double n_y = std::cos(p0.theta);
  st.x = p0.x + 3.0 * n_x;
  st.y = p0.y + 3.0 * n_y;
  st.theta = NormalizeAngle(p0.theta - 0.12);

  size_t hint_idx = 0;
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

  const double total_time = ref.back().t + 8.0;
  const int max_steps = static_cast<int>(total_time / ts);
  const int frame_stride = 10;

  plt::backend("Agg");
  plt::figure_size(1200, 900);

  int frame_id = 0;
  for (int k = 0; k < max_steps; ++k) {
    const double sim_t = k * ts;
    const RefPoint target_t = InterpolateByTime(ref, sim_t);
    const FrenetMatch m = MatchToTrajectory(ref, st, hint_idx);
    hint_idx = m.idx;
    st.s = m.s;

    const double station_error = target_t.s - st.s;
    const double speed_error = target_t.v - st.v;

    const double speed_offset = station_pid.Control(station_error, ts);
    const double speed_input = Clamp(speed_offset + speed_error, -3.0, 3.0);
    double a_cmd = speed_pid.Control(speed_input, ts) + target_t.a;

    if (st.s > ref.back().s - 1.2 && st.v > 0.15) {
      a_cmd = std::min(a_cmd, -1.2);
    }
    a_cmd = Clamp(a_cmd, -4.0, 2.5);

    const double v_for_lqr = std::max(1.0, st.v);
    A(1, 1) = A_coeff(1, 1) / v_for_lqr;
    A(1, 3) = A_coeff(1, 3) / v_for_lqr;
    A(3, 1) = A_coeff(3, 1) / v_for_lqr;
    A(3, 3) = A_coeff(3, 3) / v_for_lqr;
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
    Ad = (I - ts * 0.5 * A).inverse() * (I + ts * 0.5 * A);

    SolveLQRProblem(Ad, Bd, Q, R, 0.01, 200, &K);

    const double e_y = m.d;
    const double e_psi = NormalizeAngle(st.theta - m.ref.theta);
    const double e_y_dot = st.v * std::sin(e_psi);
    const double e_psi_dot = st.omega - m.ref.kappa * m.ref.v;

    Eigen::MatrixXd x_state = Eigen::MatrixXd::Zero(4, 1);
    x_state(0, 0) = e_y;
    x_state(1, 0) = e_y_dot;
    x_state(2, 0) = e_psi;
    x_state(3, 0) = e_psi_dot;

    const double delta_fb = -(K * x_state)(0, 0);
    const double kv = lr * mass / (2.0 * cf * wheelbase) -
                      lf * mass / (2.0 * cr * wheelbase);
    const double delta_ff = wheelbase * m.ref.kappa +
                            kv * st.v * st.v * m.ref.kappa -
                            K(0, 2) * (lr * m.ref.kappa -
                                        lf * mass * st.v * st.v * m.ref.kappa /
                                            (2.0 * cr * wheelbase));

    const double delta_cmd = Clamp(delta_fb + delta_ff, -0.5, 0.5);

    st.steer = delta_cmd;
    st.a = a_cmd;
    st.v = std::max(0.0, st.v + st.a * ts);
    st.omega = st.v / wheelbase * std::tan(st.steer);
    st.theta = NormalizeAngle(st.theta + st.omega * ts);
    st.x += st.v * std::cos(st.theta) * ts;
    st.y += st.v * std::sin(st.theta) * ts;

    traj_x.push_back(st.x);
    traj_y.push_back(st.y);

    if (k % frame_stride == 0) {
      plt::clf();
      plt::xlim(-102.0, 40.0);
      plt::ylim(-40.0, 40.0);
      plt::grid(true);
      plt::axis("equal");

      plt::plot(ref_x, ref_y, {{"label", "Reference path"}, {"color", "black"}, {"linestyle", "--"}});
      plt::plot(traj_x, traj_y, {{"label", "Vehicle trajectory"}, {"color", "blue"}, {"linestyle", "-"}});

      PlotVehicleBox(st, wheelbase);

      plt::scatter(std::vector<double>{m.ref.x}, std::vector<double>{m.ref.y},
                   60.0, {{"color", "orange"}, {"label", "Lateral target"}});
      plt::scatter(std::vector<double>{target_t.x}, std::vector<double>{target_t.y},
                   60.0, {{"color", "magenta"}, {"label", "Longitudinal target"}});
      plt::scatter(std::vector<double>{st.x}, std::vector<double>{st.y},
                   40.0, {{"color", "black"}, {"label", "Rear axle"}});

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << "LATERAL LQR (dynamic model)\n"
          << "Q=[0.01,0,0.05,0], R=0.05\n"
          << "e_y=" << e_y << " m, e_psi=" << e_psi << " rad\n"
          << "delta_fb=" << delta_fb << " rad, delta_ff=" << delta_ff
          << " rad\n\n"
          << "LONGITUDINAL PID (dual-loop)\n"
          << "Station PID: Kp=15 Ki=0.2 Kd=0.01\n"
          << "Speed PID: Kp=1.5 Ki=0.5 Kd=0.0\n"
          << "s=" << st.s << " m, v=" << st.v << " m/s, a_cmd=" << a_cmd;
      plt::text(-106.0, 43.0, oss.str());

      plt::legend("lower left");

      std::ostringstream filename;
      filename << "output/frames/frame_" << std::setfill('0') << std::setw(4)
               << frame_id++ << ".png";
      plt::save(filename.str());
    }

    if (st.s >= ref.back().s - 0.2 && st.v < 0.05) {
      break;
    }
  }

  const int gif_ret = std::system(
      "python3 scripts/make_gif.py output/frames output/demo.gif 12");
  if (gif_ret != 0) {
    std::cerr << "Failed to generate gif automatically. Run: "
              << "python3 scripts/make_gif.py output/frames output/demo.gif 12\n";
  }

  std::cout << "Simulation finished. GIF: output/demo.gif\n";
  return 0;
}
