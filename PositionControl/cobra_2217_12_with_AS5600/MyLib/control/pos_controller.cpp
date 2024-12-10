#include "pos_controller.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
#include "motorcontrol.h"
#ifdef __cplusplus
 }
#endif /* __cplusplus */

PosController::PosController():
  p_gain_(0.1f),
  i_gain_(0.01f),
  d_gain_(0.0f),
  target_p_(2048),
  target_d_(0),
  torque_reach_dur_(100)
{
}

void PosController::init(MagEncoder *mag_encoder)
{
  mag_encoder_ = mag_encoder;
  last_update_time_ = HAL_GetTick();
  MC_AcknowledgeFaultMotor1();
  MC_ProgramSpeedRampMotor1(6000/6,1000);
  MC_StartMotor1();
  HAL_Delay(10000);
  MC_StopMotor1();
  HAL_Delay(1000);
}
void PosController::update()
{
  uint32_t dur = HAL_GetTick() - last_update_time_;
  if( dur >= POS_CONTROL_DUR)
    {
      last_update_time_ = HAL_GetTick();
      err_p_ = target_p_ -  mag_encoder_->getAngle();
      err_i_ += err_p_ * dur;
      err_d_ = err_p_ / dur;
      // target_final_torque_ = err_p_ * p_gain_ + err_i_ * i_gain_ + err_d_ * d_gain_;
      target_final_torque_f_ = err_p_ * p_gain_;
      target_final_torque_t_ = (int16_t)(target_final_torque_f_);
      target_final_torque_ = target_final_torque_t_;
      sendTorqueCommand();
    }
}

void PosController::sendTorqueCommand()
{
  // MC_ProgramTorqueRampMotor1(target_final_torque_, torque_reach_dur_);
}
