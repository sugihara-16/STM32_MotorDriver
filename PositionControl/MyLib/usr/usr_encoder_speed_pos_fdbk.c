/* Includes ------------------------------------------------------------------*/
#include "usr_encoder_speed_pos_fdbk.h"
#include "mc_type.h"

/*Mag Encoder Instance*/
MagEncoderCwrap* mag_encoder = NULL;

#define ABS_ENCODER_RESOLUTION    (4096)  /* 12-bit resolution */
#define FULL_SCALE_ANGLE          (32767) /* 360.00° expressed in hundredths */

void ENC_Init(ENCODER_Handle_t *pHandle)
{
  /*Setup mag encoder (AS5600)*/
  mag_encoder = create_encoder();
  HAL_Delay(10);
  MagCwrap_Init(mag_encoder,&hi2c1);
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* 【変更】絶対エンコーダ用として、パルス数をセンサ分解能に固定 */
    pHandle->PulseNumber = ABS_ENCODER_RESOLUTION;
    pHandle->U32MAXdivPulseNumber = UINT32_MAX / ((uint32_t) pHandle->PulseNumber);
    pHandle->SpeedSamplingFreqUnit = ((uint32_t)pHandle->SpeedSamplingFreqHz * (uint32_t)SPEED_UNIT);

    /* Clear the speed (delta) buffer */
    {
      uint8_t index;
      for (index = 0U; index < pHandle->SpeedBufferSize; index++)
      {
        pHandle->DeltaCapturesBuffer[index] = 0;
      }
    }
    /* 【変更】絶対エンコーダの場合、タイマーは使用せず updateEncoder()/readMagEncoder() で初期角度を取得 */
    {
      /* まず updateEncoder() を呼び出して、最新値を取得（周期的な呼び出しが前提） */
      /* updateEncoder();  /\* 【追加】 *\/ */
      /* uint16_t raw = readMagEncoder();  /\* 【変更】I2C経由で取得した値を利用 *\/ */
      /* MagCwrap_Update(mag_encoder); */
      uint32_t raw = MagCwrap_GetAngle(mag_encoder);
      /* Convert raw 12-bit value (0..4095) to an angle in hundredths of degree */
      int32_t angle = (int32_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION);  /* 【変更】換算処理 */
      pHandle->PreviousCapture = angle;
      pHandle->_Super.hMecAngle = angle;
      pHandle->_Super.hElAngle = angle * (int16_t)(pHandle->_Super.bElToMecRatio);
      pHandle->_Super.wMecAngle = (((int32_t)angle) * 2)/* %65536 */;
    }
    pHandle->SensorIsReliable = true;
    pHandle->TimerOverflowError = false; /* 【変更】タイマーオーバーフロー処理不要 */
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Clear software FIFO where the captured rotor angle variations are stored.
  *         This function must be called before starting the motor to initialize
  *         the speed measurement process.
  * @param  pHandle: handler of the current instance of the encoder component
  */
void ENC_Clear(ENCODER_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint8_t index;
    for (index = 0U; index < pHandle->SpeedBufferSize; index++)
    {
      pHandle->DeltaCapturesBuffer[index] = 0;
    }
    {
      /* updateEncoder();  /\* 【追加】更新 *\/ */
      /* uint16_t raw = readMagEncoder();  /\* 【変更】絶対エンコーダから値を取得 *\/ */
      /* MagCwrap_Update(mag_encoder); */
      uint16_t raw = MagCwrap_GetAngle(mag_encoder);
      int16_t angle = (int16_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION);  /* 【変更】角度に換算 */
      pHandle->PreviousCapture = angle;
    }
    pHandle->SensorIsReliable = true;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

int16_t ENC_magUpdate(ENCODER_Handle_t *pHandle)
{
  MagCwrap_Update(mag_encoder);
}

/**
  * @brief  It calculates the rotor electrical and mechanical angle based on the
  *         current absolute encoder reading.
  * @param  pHandle: handler of the current instance of the encoder component
  * @retval Measured electrical angle in [s16degree] format.
  */
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    elAngle = 0;
  }
  else
  {
#endif
    /* 【変更】絶対エンコーダの場合、タイマーから読み出すのではなく updateEncoder() → readMagEncoder() で取得 */
    uint16_t raw = MagCwrap_GetAngle(mag_encoder);
    /* Convert raw value to hundredths of degree: 0..4095 -> 0..35999 */
    int16_t mecAngle = (int32_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION);  /* 【変更】絶対値の換算 */

    /* Compute the incremental angle (with wrap-around correction) */
    int16_t prevMecAngle = pHandle->_Super.hMecAngle;
    int16_t delta = mecAngle - prevMecAngle;
    if(delta > (FULL_SCALE_ANGLE / 2))
      {
        delta -=FULL_SCALE_ANGLE;
      }
    else if(delta < -(FULL_SCALE_ANGLE / 2))
      {
        delta += FULL_SCALE_ANGLE;
      }
    /* Update the accumulated mechanical angle */
    pHandle->_Super.wMecAngle += ((int32_t)delta)*2;
    /* pHandle->_Super.wMecAngle %= 65536; */

    /* Store current mechanical angle */
    pHandle->_Super.hMecAngle = mecAngle;
    /* Compute electrical angle using conversion ratio */
    int16_t elAngle = mecAngle * (int16_t)(pHandle->_Super.bElToMecRatio);
    pHandle->_Super.hElAngle = elAngle;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
  return elAngle;
}

/**
  * @brief  This method must be called with the periodicity defined by parameter
  *         SpeedSamplingFreqUnit. The method reads the absolute encoder value,
  *         computes and stores average mechanical speed (in the unit defined by SPEED_UNIT),
  *         average mechanical acceleration, and instantaneous electrical speed in dpp,
  *         updates the speed buffer and returns the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  pMecSpeedUnit pointer used to return the rotor average mechanical speed.
  * @retval true = sensor information is reliable. false = sensor information is not reliable.
  */
bool ENC_CalcAvrgMecSpeedUnit(ENCODER_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
  bool bReliability;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if ((NULL == pHandle) || (NULL == pMecSpeedUnit))
  {
    bReliability = false;
  }
  else
  {
#endif
    int32_t wOverallAngleVariation = 0;
    uint8_t bBufferSize = pHandle->SpeedBufferSize;
    uint8_t bBufferIndex;

    /* 【変更】絶対エンコーダの場合、毎回 updateEncoder()→readMagEncoder() で現在角度を取得 */
    uint32_t raw = MagCwrap_GetAngle(mag_encoder);
    int32_t currentAngle = (int16_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION) * 2;  /* 【変更】角度換算 */

    /* Compute delta from previous capture with wrap-around correction */
    int32_t delta = currentAngle - pHandle->PreviousCapture;
    if(delta > (FULL_SCALE_ANGLE / 2))
      delta -= FULL_SCALE_ANGLE;
    else if(delta < -(FULL_SCALE_ANGLE / 2))
      delta += FULL_SCALE_ANGLE;

    /* Store delta in the circular buffer */
    pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] = delta;
    /* Update previous capture */
    pHandle->PreviousCapture = currentAngle;

    /* Sum all delta samples */
    for (bBufferIndex = 0U; bBufferIndex < bBufferSize; bBufferIndex++)
    {
      wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
    }
    /* Compute average mechanical speed:
       average speed = (total angle change * SpeedSamplingFreqUnit) / (PulseNumber * BufferSize) */
    int32_t wtemp1 = wOverallAngleVariation * ((int32_t) pHandle->SpeedSamplingFreqUnit);
    int32_t wtemp2 = ((int32_t) pHandle->PulseNumber) * ((int32_t) bBufferSize);
    if (wtemp2 != 0)
      wtemp1 = wtemp1 / wtemp2;

    *pMecSpeedUnit = (int16_t)wtemp1;

    /* Compute average mechanical acceleration */
    pHandle->_Super.hMecAccelUnitP = (int16_t)(wtemp1 - pHandle->_Super.hAvrMecSpeedUnit);
    /* Store average speed */
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wtemp1;

    /* Compute instantaneous electrical speed [dpp] using current sample delta:
       inst_speed = (delta * SpeedSamplingFreqHz * bElToMecRatio / PulseNumber)
                    * DPPConvFactor / hMeasurementFrequency  */
    int32_t instSpeed = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] *
                        ((int32_t) pHandle->SpeedSamplingFreqHz) *
                        ((int32_t) pHandle->_Super.bElToMecRatio);
    instSpeed /= ((int32_t) pHandle->PulseNumber);
    instSpeed *= ((int32_t) pHandle->_Super.DPPConvFactor);
    instSpeed /= ((int32_t) pHandle->_Super.hMeasurementFrequency);
    pHandle->_Super.hElSpeedDpp = (int16_t)instSpeed;

    /* Update buffer index */
    pHandle->DeltaCapturesIndex++;
    if (pHandle->DeltaCapturesIndex >= bBufferSize)
    {
      pHandle->DeltaCapturesIndex = 0U;
    }

    /* For absolute encoder, we assume sensor is reliable */
    bReliability = true;
    pHandle->SensorIsReliable = true;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
  return (bReliability);
}

/**
  * @brief  It sets the instantaneous rotor mechanical angle.
  *         For an absolute encoder, this updates the software state.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle in [s16degree] format.
  */
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
r  {
#endif
    pHandle->_Super.hMecAngle = hMecAngle;
    pHandle->_Super.hElAngle = (int16_t)(hMecAngle * pHandle->_Super.bElToMecRatio);
    /* 【変更】タイマー操作は不要なため、PreviousCaptureも更新して差分計算のジャンプを防止 */
    pHandle->PreviousCapture = hMecAngle;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Absolute encoder does not use timer overflow interrupts.
  *         This handler is provided for compatibility and does nothing.
  * @param  pHandleVoid: handler of the current instance of the encoder component
  * @retval Always returns MC_NULL.
  */
void *ENC_IRQHandler(void *pHandleVoid)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandleVoid)
    {
      return (MC_NULL);
    }
#endif
  /* 【変更】絶対エンコーダではタイマーオーバーフロー割込みは発生しないため、何も処理しない */
  return (MC_NULL);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  MagCwap_DmaCallback(mag_encoder);
}
