#ifndef MAG_ENCODER_H_
#define MAG_ENCODER_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

class MagEncoder {
public:
    MagEncoder();

    HAL_StatusTypeDef init(I2C_HandleTypeDef *i2cHandle);

    HAL_StatusTypeDef update();

    uint16_t getAngle() const;
    uint16_t getRawAngle() const;
    uint16_t getMagneticMagnitude() const;
    uint8_t getAGC() const;
    bool isMagnetDetected() const;

private:
    I2C_HandleTypeDef *i2cHandle_;
    uint8_t i2cAddr_;
    uint32_t i2c_error_code_;

    uint32_t last_time_;
    uint16_t device_id_;

    uint16_t angle_;
    uint16_t rawAngle_;
    uint16_t magneticMagnitude_;
    uint8_t agc_;
    bool magnetDetected_;

    HAL_StatusTypeDef readRegister(uint8_t regAddr, uint8_t *data, uint16_t len);
    HAL_StatusTypeDef writeRegister(uint8_t regAddr, uint8_t *data, uint16_t len);
};

#endif // MAG_ENCODER_H_
