#ifndef MAG_ENCODER_WRAPPER_H
#define MAG_ENCODER_WRAPPER_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct MagEncoderCwrap MagEncoderCwrap;

  MagEncoderCwrap* create_encoder();
  void MagCwrap_Init(MagEncoderCwrap* mag_wrapper, I2C_HandleTypeDef* i2cHandle);
  void MagCwrap_Update(MagEncoderCwrap* mag_wrapper);
  uint16_t MagCwrap_GetAngle(MagEncoderCwrap* mag_wrapper);
  // void destroy_encoder(MagEncoderCwrap* wrapper);
  // int get_encoder_position(MagEncoderCwrap* wrapper);

#ifdef __cplusplus
    }
#endif

#endif // MAG_ENCODER_WRAPPER_H
