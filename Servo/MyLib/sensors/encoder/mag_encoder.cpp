#include "mag_encoder.h"

MagEncoder::MagEncoder() 
  : i2cHandle_(nullptr),
    i2cAddr_((AS5600_SLAVE_ADDRESS & 0x7F) << 1),
    angle_(0), rawAngle_(0), magneticMagnitude_(0),
    agc_(0), magnetDetected_(false), last_time_(0), device_id_(0),
    dmaState_(DMA_IDLE)
{
}

HAL_StatusTypeDef MagEncoder::init(I2C_HandleTypeDef *i2cHandle) {
  i2cHandle_ = i2cHandle;
  last_time_ = HAL_GetTick() + 6000;
  device_id_ = 128;
  dmaState_ = DMA_IDLE;
  return HAL_OK;
}

HAL_StatusTypeDef MagEncoder::update() {
  if(dmaState_ != DMA_IDLE && dmaState_ != DMA_COMPLETE) {
    return HAL_BUSY;
  }
  dmaState_ = DMA_READING_STATUS;
  // Transmit i2c read command (1st time)
  return HAL_I2C_Mem_Read_DMA(i2cHandle_,
                              i2cAddr_,
                              AS5600_STATUS_REG_ADDR,
                              I2C_MEMADD_SIZE_8BIT,
                              dmaBuffer1_,
                              5);
}

void MagEncoder::DMA_ReadCompleteCallback() {
  if(dmaState_ == DMA_READING_STATUS) {
    // dmaBuffer1_[0]: STATUS
    // dmaBuffer1_[1-2]: RAW ANGLE (MSB, LSB)
    // dmaBuffer1_[3-4]: ANGLE (MSB, LSB)
    magnetDetected_ = (dmaBuffer1_[0] & (1 << 5)) != 0;
    rawAngle_       = (dmaBuffer1_[1] << 8) | dmaBuffer1_[2];
    angle_          = (dmaBuffer1_[3] << 8) | dmaBuffer1_[4];

    // Transmit i2c read command (2nd time)
    dmaState_ = DMA_READING_AGC_MAG;
    if(HAL_I2C_Mem_Read_DMA(i2cHandle_,
                            i2cAddr_,
                            AS5600_AGC_REG_ADDR,
                            I2C_MEMADD_SIZE_8BIT,
                            dmaBuffer2_,
                            3) != HAL_OK)
    {
      dmaState_ = DMA_ERROR;
    }
  }
  else if(dmaState_ == DMA_READING_AGC_MAG) {
    agc_ = dmaBuffer2_[0];
    magneticMagnitude_ = (dmaBuffer2_[1] << 8) | dmaBuffer2_[2];
    dmaState_ = DMA_COMPLETE;
  }
}

void MagEncoder::DMA_ErrorCallback() {
  dmaState_ = DMA_ERROR;
}

uint16_t MagEncoder::getAngle() const {
  return angle_;
}

uint16_t MagEncoder::getRawAngle() const {
  return rawAngle_;
}

uint16_t MagEncoder::getMagneticMagnitude() const {
  return magneticMagnitude_;
}

uint8_t MagEncoder::getAGC() const {
  return agc_;
}


bool MagEncoder::isMagnetDetected() const {
  return magnetDetected_;
}

HAL_StatusTypeDef MagEncoder::readRegister(uint8_t regAddr, uint8_t *data, uint16_t len) {
  return HAL_I2C_Mem_Read(i2cHandle_, i2cAddr_, regAddr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MagEncoder::writeRegister(uint8_t regAddr, uint8_t *data, uint16_t len) {
  return HAL_I2C_Mem_Write(i2cHandle_, i2cAddr_, regAddr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
