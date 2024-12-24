#ifndef POS_CONTROLLER_H_
#define POS_CONTROLLER_H_

#define POS_CONTROL_DUR 1 //100hz
#define MOTOR_STACK_CHECK_DUR 500 //100hz

#include <cassert>
#include <algorithm>
#include <functional>
#include "sensors/encoder/mag_encoder.h"


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
#include "motorcontrol.h"
#ifdef __cplusplus
 }
#endif /* __cplusplus */

class PosController {
public:
  PosController();
  ~PosController(){};

  void init(MagEncoder *mag_encoder);

  void MotorInit();

  void update();

  void sendTorqueCommand();

  void rebootMotor();

private:
  //instancesxo
  MagEncoder *mag_encoder_;
  //variables for position control
  float p_gain_, i_gain_, d_gain_;
  uint16_t target_p_, target_d_;
  float err_p_, err_i_, err_d_;
  float err_i_lim_;
  uint32_t torque_lim_;
  uint32_t last_update_time_;
  int16_t current_rmp_;

  //for motor stack detection
  uint32_t last_stack_check_time_;
  float previous_err_p_;

  //variables for torque command for FOC
  int16_t target_final_torque_;
  uint16_t torque_reach_dur_;

  State_t motor_state_;
  uint16_t motor_crr_fault_code_;
  uint16_t motor_past_fault_code_;

  int16_t rot_dir_;
  bool start_flag_;
};

#endif // POS_CONTROLLER_H_
