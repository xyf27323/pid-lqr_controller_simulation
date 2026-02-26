#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

struct PidConfig {
  bool integrator_enable = false;
  double integrator_saturation_level = 0.0;
  double output_saturation_level = 0.0;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double kaw = 0.0;
};

class PIDController {
 public:
  void Init(const PidConfig& pid_conf);
  void SetPID(const PidConfig& pid_conf);
  void Reset();
  double Control(double error, double dt);
  virtual ~PIDController() = default;

  int IntegratorSaturationStatus() const;
  bool IntegratorHold() const;
  void SetIntegratorHold(bool hold);

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;

  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;

  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
};

#endif
