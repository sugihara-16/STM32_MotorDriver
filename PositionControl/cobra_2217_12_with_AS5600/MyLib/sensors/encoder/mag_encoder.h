/*
******************************************************************************
* File Name          : mag_encoder.h
* Description        : Magnetic Encoder Interface
******************************************************************************
*/
#include "stm32g4xx_hal.h"
#include "stdint.h"

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

class MagEncoder
{
public:
  MagEncoder(){};
  ~MagEncoder(){};
  
  static const uint8_t AS5600_I2C_ADDRESS =  0x36 << 1; // 0x36 << 1; NOTE: STM: i2c_address << 1 !!!
  static const uint8_t AS5600_REG_RAW_ANGLE =  0x0C;
  static const uint8_t UPDATE_INTERVAL = 20; //20 -> 50Hz

  void init(I2C_HandleTypeDef* hi2c);
  void update(void);

private:
  I2C_HandleTypeDef* hi2c_;

  uint16_t raw_encoder_value_;
  uint32_t last_time_;

  uint32_t i2c_error_code_;

};

