#include "encoder_speed_pos_fdbk.h"
#include "sensors/encoder/mag_encoder.h"

#ifndef __USR_ENCODER_SPEED_POS_FDBK_H
#define __USR_ENCODER_SPEED_POS_FDBK_H

extern MagEncoder mag_encoder;
extern I2C_HandleTypeDef hi2c1;
void * ENC_IRQHandler( void * pHandleVoid );
void ENC_Init( ENCODER_Handle_t * pHandle );
void ENC_Clear( ENCODER_Handle_t * pHandle );
int16_t ENC_CalcAngle( ENCODER_Handle_t * pHandle );
bool ENC_CalcAvrgMecSpeedUnit( ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit );
void ENC_SetMecAngle( ENCODER_Handle_t * pHandle, int16_t hMecAngle );
void ENC_updateMagEncoder( ENCODER_Handle_t * pHandle);
#endif // __USR_ENCODER_SPEED_POS_FDBK_H
