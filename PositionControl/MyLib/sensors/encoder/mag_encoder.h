#ifndef MAG_ENCODER_H_
#define MAG_ENCODER_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define AS5600_SLAVE_ADDRESS      0x36
#define AS5600_RAWANGLE_REG_ADDR  0x0C
#define AS5600_ANGLE_REG_ADDR     0x0E
#define AS5600_STATUS_REG_ADDR    0x0B
#define AS5600_AGC_REG_ADDR       0x1A
#define AS5600_MAG_REG_ADDR       0x1B

class MagEncoder {
public:
  MagEncoder();

  HAL_StatusTypeDef init(I2C_HandleTypeDef *i2cHandle);
  HAL_StatusTypeDef update();
  void DMA_ReadCompleteCallback();
  void DMA_ErrorCallback();

  uint16_t getAngle() const;
  uint16_t getRawAngle() const;
  uint16_t getMagneticMagnitude() const;
  uint8_t  getAGC() const;
  bool     isMagnetDetected() const;

  I2C_HandleTypeDef* getI2CHandle() const { return i2cHandle_; }

private:
  HAL_StatusTypeDef readRegister(uint8_t regAddr, uint8_t *data, uint16_t len);
  HAL_StatusTypeDef writeRegister(uint8_t regAddr, uint8_t *data, uint16_t len);

  I2C_HandleTypeDef* i2cHandle_;
  uint16_t i2cAddr_;

  uint16_t angle_;
  uint16_t rawAngle_;
  uint16_t magneticMagnitude_;
  uint8_t  agc_;
  bool     magnetDetected_;

  uint32_t last_time_;
  uint8_t  device_id_;

  uint8_t dmaBuffer1_[5]; //i2c resistor 0x0B～0x0F: STATUS, RAWANGLE (2byte), ANGLE (2byte)
  uint8_t dmaBuffer2_[3]; //i2c resistor AGC (1byte), MAGNETIC (2byte)

  enum DMAState {
    DMA_IDLE,
    DMA_READING_STATUS,
    DMA_READING_AGC_MAG,
    DMA_COMPLETE,
    DMA_ERROR
  } dmaState_;
};

#endif  // MAG_ENCODER_H
