#ifndef LQR_CONTROLLER_H_
#define LQR_CONTROLLER_H_

#include "Eigen/Core"
#include "sim_types.h"

struct LqrDebug {
  double e_y = 0.0;
  double e_psi = 0.0;
  double delta_fb = 0.0;
  double delta_ff = 0.0;
};

class LqrController {
 public:
  struct Params {
    double ts = 0.01;
    double wheelbase = 2.72;

    // 车辆横摆动力学参数。
    double cf = 155494.663;
    double cr = 155494.663;
    double mass_fl = 520.0;
    double mass_fr = 520.0;
    double mass_rl = 520.0;
    double mass_rr = 520.0;

    // LQR 代价函数参数。
    double q11 = 0.01;
    double q22 = 0.0;
    double q33 = 0.05;
    double q44 = 0.0;
    double r = 0.05;

    double lqr_eps = 0.01;
    unsigned int lqr_max_iter = 200;
    double minimum_speed_protection = 1.0;
    double steer_limit = 0.5;
  };

  LqrController();
  explicit LqrController(const Params& params);

  double ComputeSteering(const VehicleState& st,
                         const FrenetMatch& match,
                         LqrDebug* debug) const;

 private:
  Params p_;

  double mass_ = 0.0;
  double lf_ = 0.0;
  double lr_ = 0.0;
  double iz_ = 0.0;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd A_coeff_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd Bd_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

#endif
