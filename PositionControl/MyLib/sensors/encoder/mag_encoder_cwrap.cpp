#include "mag_encoder_cwrap.h"
#include "mag_encoder.h"

struct MagEncoderCwrap {
  MagEncoder* encoder;
};

extern "C" {
  MagEncoderCwrap* create_encoder()
  {
    MagEncoderCwrap* wrapper = new MagEncoderCwrap();
    wrapper->encoder = new MagEncoder();
    return wrapper;
  }

  void MagCwrap_Init(MagEncoderCwrap* mag_wrapper, I2C_HandleTypeDef* i2cHandle) {
    if (mag_wrapper && mag_wrapper->encoder) {
      mag_wrapper->encoder->init(i2cHandle);
    }
  }

  void MagCwrap_Update(MagEncoderCwrap* mag_wrapper) {
    if (mag_wrapper && mag_wrapper->encoder) {
      mag_wrapper->encoder->update();
    }
  }

  void MagCwap_DmaCallback(MagEncoderCwrap* mag_wrapper) {
    if (mag_wrapper && mag_wrapper->encoder) {
      mag_wrapper->encoder->DMA_ReadCompleteCallback();
    }
  }

  uint16_t MagCwrap_GetAngle(MagEncoderCwrap* mag_wrapper) {
    uint16_t angle;
    if (mag_wrapper && mag_wrapper->encoder) {      
      angle = mag_wrapper->encoder->getAngle();
    }
    return angle;
  }
}
