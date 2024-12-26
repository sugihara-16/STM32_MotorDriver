#include "encoder_speed_pos_fdbk.h"
#include "sensors/encoder/mag_encoder.h"

void * ENC_IRQHandler( void * pHandleVoid );
void ENC_Init( ENCODER_Handle_t * pHandle );
void ENC_Clear( ENCODER_Handle_t * pHandle );
int16_t ENC_CalcAngle( ENCODER_Handle_t * pHandle );
bool ENC_CalcAvrgMecSpeedUnit( ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit );
void ENC_SetMecAngle( ENCODER_Handle_t * pHandle, int16_t hMecAngle );
