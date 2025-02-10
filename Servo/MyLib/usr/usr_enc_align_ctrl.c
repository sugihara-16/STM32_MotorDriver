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
        /*get and store current absolute angle*/
        ENC_magUpdate(pHandle->pENC);
        while(ENC_magDmaReadCheck(pHandle->pENC));
        ENC_CalcAngle(pHandle->pENC);
        ENC_SetMecZeroOffset(pHandle->pENC, aligned_mecAngle);//Very important! This process ensure the proper conversion from absolute angle to electrical angle.
        ENC_SetInitAbsPos(pHandle->pENC, pHandle->pENC->_Super.wMecAngle);
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
