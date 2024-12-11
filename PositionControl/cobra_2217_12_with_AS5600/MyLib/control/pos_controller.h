#ifndef POS_CONTROLLER_H_
#define POS_CONTROLLER_H_

#define POS_CONTROL_DUR 10 //100hz

#include <cassert>
#include <algorithm>
#include <functional>
#include "sensors/encoder/mag_encoder.h"

class PosController {
public:
  PosController();
  ~PosController(){};

  void init(MagEncoder *mag_encoder);

  void update();

  void sendTorqueCommand();

private:
  //instancesxo
  MagEncoder *mag_encoder_;
  //variables for position control
  float p_gain_, i_gain_, d_gain_;
  uint16_t target_p_, target_d_;
  float err_p_, err_i_, err_d_;
  float err_i_lim_;
  uint32_t last_update_time_;

  //variables for torque command for FOC
  int16_t target_final_torque_;
  uint16_t torque_reach_dur_;
};

#endif // POS_CONTROLLER_H_
