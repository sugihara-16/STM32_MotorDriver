/*
******************************************************************************
* File Name          : mag_encoder.cpp
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "mag_encoder.h"

void MagEncoder::init(I2C_HandleTypeDef* hi2c)
{
  hi2c_ = hi2c;
  raw_encoder_value_ = 0;

  last_time_ = HAL_GetTick() + 6000; // after 6s
}

void MagEncoder::update(void)
{
  uint32_t now_time = HAL_GetTick();
  if(now_time >= last_time_ + UPDATE_INTERVAL)
    {
      last_time_ = now_time;
      uint16_t val[1];
      val[0] = AS5600_REG_RAW_ANGLE;
      int i2c_status = HAL_I2C_Master_Transmit(hi2c_, AS5600_I2C_ADDRESS, reinterpret_cast<uint8_t*>(val), 1, 100);
      if(i2c_status == HAL_OK)
        {
          uint16_t adc[2];
          HAL_I2C_Master_Receive(hi2c_, AS5600_I2C_ADDRESS , reinterpret_cast<uint8_t*>(adc), 2, 100);
          raw_encoder_value_ = (uint16_t)(adc[0] << 8 | adc[1]);
        }
      else
        {
          raw_encoder_value_ = 65535;
        }
    }

}