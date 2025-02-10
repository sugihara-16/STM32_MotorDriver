// #include "pos_controller.h"

// PosController::PosController():
//   p_gain_(0.1f),
//   i_gain_(0.1f),
//   d_gain_(0.1f),
//   target_p_(2048),
//   target_d_(0),
//   err_i_lim_(10000.0f),
//   torque_reach_dur_(100),
//   torque_lim_(3000),
//   previous_err_p_(0.0f),
//   current_rmp_(0)
// {
// }

// void PosController::init(MagEncoder *mag_encoder)
// {
//   mag_encoder_ = mag_encoder;
//   last_update_time_ = HAL_GetTick();
//   // MotorInit();
//   rot_dir_ = -1;
//   start_flag_ = false;
//   MC_AcknowledgeFaultMotor1();
//   MC_ProgramTorqueRampMotor1(0,0);
//   MC_StartMotor1();
//   motor_crr_fault_code_ = 0;
//   motor_past_fault_code_ = 0;
// }

// void PosController::MotorInit()
// {
//   while (MC_GetSTMStateMotor1() != IDLE)
//     {
//       // motor_fault_code_ = MC_GetCurrentFaultsMotor1();
//       MC_AcknowledgeFaultMotor1();
//     }
//   while (!MC_StartMotor1())
//     {
//       // motor_fault_code_ = MC_GetCurrentFaultsMotor1();
//       MC_AcknowledgeFaultMotor1();
//     }
//   MC_ProgramTorqueRampMotor1(0,0);
// }
// void PosController::update()
// {
//   motor_state_ = MC_GetSTMStateMotor1();
//   uint32_t stack_dur = HAL_GetTick() - last_stack_check_time_;
//   uint32_t dur = HAL_GetTick() - last_update_time_;
//   if( dur >= POS_CONTROL_DUR)
//     {
//       last_update_time_ = HAL_GetTick();
//       err_p_ = target_p_ -  mag_encoder_->getAngle();
//       err_i_ = std::max(err_i_ + err_p_ * dur, err_i_lim_);
//       current_rmp_ = MC_GetMecSpeedAverageMotor1();
//       // err_d_ = target_d_ - current_rmp_;
//       // target_final_torque_ = err_p_ * p_gain_ + err_i_ * i_gain_ + err_d_ * d_gain_;
//       target_final_torque_ = (int16_t)(err_p_ * p_gain_ + err_d_ * d_gain_);
//       sendTorqueCommand();
//     }
//   // if(stack_dur >= MOTOR_STACK_CHECK_DUR)
//   //   {
//   //     if(std::abs(err_p_ - previous_err_p_) < 1.0f) rebootMotor();
//   //     previous_err_p_ = err_p_;
//   //     last_stack_check_time_ = HAL_GetTick();
//   //   }
// }

// void PosController::sendTorqueCommand()
// {
//   if(motor_state_ == FAULT_OVER || motor_state_ == FAULT_NOW){
//     MC_AcknowledgeFaultMotor1();
//     MC_ProgramTorqueRampMotor1(target_final_torque_, 0);
//     MC_StartMotor1();
//     return;
//   }
//   MC_ProgramTorqueRampMotor1(target_final_torque_, 0);
// }

// void PosController::rebootMotor()
// {
//   MC_AcknowledgeFaultMotor1();
//   MC_StopMotor1();
//   MC_ProgramTorqueRampMotor1(target_final_torque_, 0);
//   MC_StartMotor1();
// }
