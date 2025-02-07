/* Includes ------------------------------------------------------------------*/
#include "usr_encoder_speed_pos_fdbk.h"
#include "mc_type.h"

/*Mag Encoder Instance*/
MagEncoderCwrap* mag_encoder = NULL;
/*Encoder offset*/
int16_t MecZeroOffset = 0;

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
    /* For an absolute encoder, we fix the pulse number to the resolution */
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
    
    pHandle->SensorIsReliable = true;
    pHandle->TimerOverflowError = false; /* Not used for absolute encoder */
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

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
      uint16_t raw = MagCwrap_GetAngle(mag_encoder);
      int16_t angle = (int16_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION);
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

bool ENC_magDmaReadCheck(ENCODER_Handle_t *pHandle)
{
  return MagCwrap_isDataReady(mag_encoder);
}

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
    uint16_t raw = MagCwrap_GetAngle(mag_encoder);
    /* Convert raw value to hundredths of degree: 0..4095 -> 0..35999 */
    int16_t mecAngle = (int32_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION) - MecZeroOffset;

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

    uint32_t raw = MagCwrap_GetAngle(mag_encoder);
    int32_t currentAngle = (int16_t)((raw * FULL_SCALE_ANGLE) / ABS_ENCODER_RESOLUTION) - MecZeroOffset;

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
    pHandle->PreviousCapture = hMecAngle;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

void ENC_SetMecZeroOffset(ENCODER_Handle_t *pHandle, int16_t AngleOffset)
{
  MecZeroOffset = AngleOffset;
}

void *ENC_IRQHandler(void *pHandleVoid)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandleVoid)
    {
      return (MC_NULL);
    }
#endif
  return (MC_NULL);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  MagCwap_DmaCallback(mag_encoder);
}
