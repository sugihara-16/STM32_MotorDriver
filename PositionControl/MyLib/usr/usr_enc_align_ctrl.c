#include "usr_enc_align_ctrl.h"
bool EAC_Exec(EncAlign_Handle_t *pHandle)
{
  bool retVal = true;
#ifdef NULL_PTR_CHECK_ENC_ALI_CTRL
  if (NULL == pHandle)
  {
    retVal = false;
  }
  else
  {
#endif
    if (pHandle->hRemainingTicks > 0U)
    {
      pHandle->hRemainingTicks--;

      if (0U == pHandle->hRemainingTicks)
      {
        /*get aligned mechanical angle*/
        int16_t aligned_mecAngle = pHandle->hElAngle / ((int16_t)pHandle->bElToMecRatio);
        /*get current absolute angle*/
        ENC_magUpdate(pHandle->pENC);
        while(ENC_magDmaReadCheck(pHandle->pENC));
        ENC_CalcAngle(pHandle->pENC);
        ENC_SetMecZeroOffset(pHandle->pENC,pHandle->pENC->_Super.hMecAngle-aligned_mecAngle);
        ENC_CalcAngle(pHandle->pENC);
        /* ENC_SetMecAngle(pHandle->pENC, (pHandle->pENC->_Super.hMecAngle)); */
        pHandle->EncAligned = true;
        retVal = true;
      }
      else
      {
        retVal = false;
      }
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_ENC_ALI_CTRL
  }
#endif

  return retVal;
}
