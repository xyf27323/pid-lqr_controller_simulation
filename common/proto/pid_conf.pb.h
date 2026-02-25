#ifndef COMMON_PROTO_PID_CONF_PB_H_
#define COMMON_PROTO_PID_CONF_PB_H_

namespace controller {

class PidConf {
 public:
  bool integrator_enable() const { return integrator_enable_; }
  double integrator_saturation_level() const {
    return integrator_saturation_level_;
  }
  double output_saturation_level() const { return output_saturation_level_; }
  double kp() const { return kp_; }
  double ki() const { return ki_; }
  double kd() const { return kd_; }
  double kaw() const { return kaw_; }

  void set_integrator_enable(bool v) { integrator_enable_ = v; }
  void set_integrator_saturation_level(double v) {
    integrator_saturation_level_ = v;
  }
  void set_output_saturation_level(double v) { output_saturation_level_ = v; }
  void set_kp(double v) { kp_ = v; }
  void set_ki(double v) { ki_ = v; }
  void set_kd(double v) { kd_ = v; }
  void set_kaw(double v) { kaw_ = v; }

 private:
  bool integrator_enable_ = false;
  double integrator_saturation_level_ = 0.0;
  double output_saturation_level_ = 0.0;
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
};

}  // namespace controller

#endif
