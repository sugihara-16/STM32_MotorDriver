#include "encoder_speed_pos_fdbk.h"
#include "sensors/encoder/mag_encoder_cwrap.h"

#ifndef __USR_ENCALIGNCTRLCLASS_H
#define __USR_ENCALIGNCTRLCLASS_H

extern I2C_HandleTypeDef hi2c1;

/*Default functions from MCSDK*/
void *ENC_IRQHandler(void *pHandleVoid);
void ENC_Init(ENCODER_Handle_t *pHandle);
void ENC_Clear(ENCODER_Handle_t *pHandle);
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle);
bool ENC_CalcAvrgMecSpeedUnit(ENCODER_Handle_t *pHandle, int16_t *pMecSpeedUnit);
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle);

/*Additional functions for absolute encoder*/
int16_t ENC_magUpdate(ENCODER_Handle_t *pHandle);
void ENC_SetMecZeroOffset(ENCODER_Handle_t *pHandle, int16_t AngleOffset);

#endif
