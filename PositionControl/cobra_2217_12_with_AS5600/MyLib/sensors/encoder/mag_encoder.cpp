#include "mag_encoder.h"

// AS5600レジスタアドレス
#define AS5600_SLAVE_ADDRESS 0x36
#define AS5600_RAWANGLE_REG_ADDR 0x0C
#define AS5600_ANGLE_REG_ADDR 0x0E
#define AS5600_STATUS_REG_ADDR 0x0B
#define AS5600_AGC_REG_ADDR 0x1A
#define AS5600_MAG_REG_ADDR 0x1B
#define UPDATE_INTERVAL 10

MagEncoder::MagEncoder()
    : i2cHandle_(nullptr), i2cAddr_((AS5600_SLAVE_ADDRESS & 0x7F) << 1),
      angle_(0), rawAngle_(0), magneticMagnitude_(0), agc_(0), magnetDetected_(false) {}

HAL_StatusTypeDef MagEncoder::init(I2C_HandleTypeDef *i2cHandle) {
    i2cHandle_ = i2cHandle;
    last_time_ = HAL_GetTick() + 6000; // after 6s
    device_id_ = 128;
    return HAL_OK;
}

HAL_StatusTypeDef MagEncoder::update() {
  uint32_t now_time = HAL_GetTick();
  if(now_time <= last_time_ + UPDATE_INTERVAL) return HAL_OK;
  last_time_ = now_time;
  for (uint16_t i = 1; i < 128; i++) {
    if (HAL_I2C_IsDeviceReady(i2cHandle_, i << 1, 1, 10) == HAL_OK) {
      device_id_ = i;
    }
  }
    uint8_t data[2] = {0};
 
    // RAW ANGLE
    if (readRegister(AS5600_RAWANGLE_REG_ADDR, data, 2) == HAL_OK) {
        rawAngle_ = (data[0] << 8) | data[1];
    }

    // ANGLE
    if (readRegister(AS5600_ANGLE_REG_ADDR, data, 2) == HAL_OK) {
        angle_ = (data[0] << 8) | data[1];
   }

    // MAGNITUDE
    if (readRegister(AS5600_MAG_REG_ADDR, data, 2) == HAL_OK) {
        magneticMagnitude_ = (data[0] << 8) | data[1];
    }

    // AGC
    if (readRegister(AS5600_AGC_REG_ADDR, data, 1) == HAL_OK) {
        agc_ = data[0];
    }

    // MAGNET STATUS
    if (readRegister(AS5600_STATUS_REG_ADDR, data, 1) == HAL_OK) {
        magnetDetected_ = (data[0] & (1 << 5)) != 0;
    }

    return HAL_OK;
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
