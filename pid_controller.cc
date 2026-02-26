#include "pid_controller.h"

#include <cmath>

double PIDController::Control(double error, double dt) {
  if (dt <= 0.0) {
    return previous_output_;
  }

  // 首帧不计算微分项，避免初值突变导致尖峰输出。
  double diff = 0.0;
  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }

  if (!integrator_enabled_) {
    integral_ = 0.0;
  } else if (!integrator_hold_) {
    integral_ += error * dt * ki_;
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
      integrator_saturation_status_ = 1;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
      integrator_saturation_status_ = -1;
    } else {
      integrator_saturation_status_ = 0;
    }
  }

  previous_error_ = error;
  const double output = error * kp_ + integral_ + diff * kd_;
  previous_output_ = output;
  return output;
}

void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
}

void PIDController::Init(const PidConfig& pid_conf) {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_enabled_ = pid_conf.integrator_enable;
  integrator_saturation_high_ = std::fabs(pid_conf.integrator_saturation_level);
  integrator_saturation_low_ = -std::fabs(pid_conf.integrator_saturation_level);
  integrator_saturation_status_ = 0;
  integrator_hold_ = false;
  SetPID(pid_conf);
}

void PIDController::SetPID(const PidConfig& pid_conf) {
  kp_ = pid_conf.kp;
  ki_ = pid_conf.ki;
  kd_ = pid_conf.kd;
  kaw_ = pid_conf.kaw;
}

int PIDController::IntegratorSaturationStatus() const {
  return integrator_saturation_status_;
}

bool PIDController::IntegratorHold() const { return integrator_hold_; }

void PIDController::SetIntegratorHold(bool hold) { integrator_hold_ = hold; }
