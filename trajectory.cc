#include "trajectory.h"

#include <cmath>
#include <limits>

std::vector<RefPoint> BuildArcTrajectory() {
  const double radius = 32.0;
  const double ds = 0.5;
  // Shorten to half of current length: from 3/4 circle to 3/8 circle.
  const double total_s = 2.0 * kPi * radius * 0.375;
  const std::size_t n = static_cast<std::size_t>(total_s / ds) + 1;
  const double phi0 = -2.35;
  const double kappa = -1.0 / radius;
  const double v_cruise = 8.0;

  std::vector<RefPoint> traj;
  traj.reserve(n);
  double t = 0.0;

  for (std::size_t i = 0; i < n; ++i) {
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
    if (progress < 0.08) {
      start_scale = std::sin(0.5 * kPi * (progress / 0.08));
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

  for (std::size_t i = 1; i + 1 < traj.size(); ++i) {
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

  std::size_t lo = 0;
  std::size_t hi = traj.size() - 1;
  while (lo + 1 < hi) {
    const std::size_t mid = (lo + hi) / 2;
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

  std::size_t lo = 0;
  std::size_t hi = traj.size() - 1;
  while (lo + 1 < hi) {
    const std::size_t mid = (lo + hi) / 2;
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
                              std::size_t hint_idx) {
  const std::size_t n = traj.size();
  const std::size_t start = (hint_idx > 30) ? hint_idx - 30 : 0;
  const std::size_t end = std::min(n - 1, hint_idx + 50);

  std::size_t best = start;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = start; i <= end; ++i) {
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
  // Continuous projection station (clamped) reduces terminal quantization.
  m.s = Clamp(ref.s + dx * cos_t + dy * sin_t, traj.front().s, traj.back().s);
  m.d = cos_t * dy - sin_t * dx;
  m.ref = InterpolateByS(traj, m.s);
  return m;
}
