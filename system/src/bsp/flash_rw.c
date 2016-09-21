#include "flash_rw.h"

static uint16_t gFlashWriteCount = 0;

void flash_write_device_id(uint32_t lDeviceID)
{
  uint32_t * pDeviceIDAddr = 0;
  pDeviceIDAddr = (uint32_t *)INFO_D_ADDR;
  __disable_interrupt();
  FCTL3 = FWKEY;
  FCTL1 = FWKEY+ERASE;
  * pDeviceIDAddr = 0;
  FCTL1 = FWKEY+BLKWRT;
  * pDeviceIDAddr = lDeviceID;
  FCTL1 = FWKEY;
  FCTL3 = FWKEY+LOCK;
  __enable_interrupt();
}

uint32_t flash_read_device_id(void)
{
  uint32_t lDeviceID = 0;
  uint32_t * pDeviceIDAddr = 0;
  pDeviceIDAddr = (uint32_t *)INFO_D_ADDR;
  lDeviceID = * pDeviceIDAddr;
  return lDeviceID;
}

#if 0
void flash_read_write_address_init(void)
{
  gFlashWriteCount = 0;
  uint8_t * pFlashSegmentSelectAddr = 0;
  uint8_t * pFlashSegmentOffset = 0;
  pFlashSegmentSelectAddr = (uint8_t *)INFO_A_ADDR;
  pFlashSegmentOffset = (uint8_t *)(INFO_A_ADDR + 1);
  __disable_interrupt();
  FCTL3 = FWKEY;
  FCTL1 = FWKEY+ERASE;
  * pFlashSegmentSelectAddr = 0;
  * pFlashSegmentOffset = 0;
  FCTL1 = FWKEY+BLKWRT;
  * pFlashSegmentSelectAddr = INFO_SEGMENT_SELECT_TYPE_A;
  * pFlashSegmentOffset = INFO_SEGMENT_OFFSET_START;
  FCTL1 = FWKEY;
  FCTL3 = FWKEY+LOCK;
  __enable_interrupt();
}

uint8_t flash_write_data(uint8_t u8Data[], uint8_t uDataLen)
{
  uint8_t uFlashSegmentSelect = 0;
  uint8_t uFlashSegmentOffset = 0;
  
  uint8_t * pFlashSegmentSelectAddr = (uint8_t *)INFO_A_ADDR;
  uint8_t * pFlashSegmentOffset = (uint8_t *)(INFO_A_ADDR + 1);
  
  uint8_t * pDataAddr = 0;
  uint8_t index = 0;
  
  if(gFlashWriteCount >= 1000)   //ÐÞ¸ÄFlash¶ÁÐ´µØÖ·
  {
    uFlashSegmentSelect = * (uint8_t *)INFO_A_ADDR;
    uFlashSegmentOffset = * (uint8_t *)(INFO_A_ADDR + 1);
    __disable_interrupt();
    FCTL3 = FWKEY;
    FCTL1 = FWKEY+ERASE;
    * pFlashSegmentSelectAddr = 0;
    * pFlashSegmentOffset = 0;
    FCTL1 = FWKEY+BLKWRT;
    * pFlashSegmentSelectAddr = (uFlashSegmentSelect + 1) % 4;
    * pFlashSegmentOffset = INFO_SEGMENT_OFFSET_START;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY+LOCK;
    __enable_interrupt();
    gFlashWriteCount  = 0;
  }
  
  uFlashSegmentSelect = * (uint8_t *)INFO_A_ADDR;
  uFlashSegmentOffset = * (uint8_t *)(INFO_A_ADDR + 1);
  
  switch(uFlashSegmentSelect)
  {
  case INFO_SEGMENT_SELECT_TYPE_A:
    pDataAddr = (uint8_t *)INFO_A_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_B:
    pDataAddr = (uint8_t *)INFO_B_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_C:
    pDataAddr = (uint8_t *)INFO_C_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_D:
    pDataAddr = (uint8_t *)INFO_D_ADDR;
    break;
  default:
    break;
  }
  
  for(index = 0; index < uDataLen; index++)
    * (uint8_t *)(pDataAddr + uFlashSegmentOffset + index) = u8Data[index];
  
  return 0;
}

uint8_t flash_read_data(uint8_t u8Data[], uint8_t uDataSize)
{
  uint8_t uFlashSegmentSelect = *(uint8_t *)INFO_A_ADDR;
  uint8_t uFlashSegmentOffset = *(uint8_t *)(INFO_A_ADDR + 1);
  uint8_t * pDataAddr = 0;
  uint8_t index = 0;
  switch(uFlashSegmentSelect)
  {
  case INFO_SEGMENT_SELECT_TYPE_A:
    pDataAddr = (uint8_t *)INFO_A_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_B:
    pDataAddr = (uint8_t *)INFO_B_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_C:
    pDataAddr = (uint8_t *)INFO_C_ADDR;
    break;
  case INFO_SEGMENT_SELECT_TYPE_D:
    pDataAddr = (uint8_t *)INFO_D_ADDR;
    break;
  default:
    break;
  }
  for(index = 0; index < uDataSize; index++)
    u8Data[index] = * (uint8_t *)(pDataAddr + uFlashSegmentOffset + index);
  
  return 0;
}
#endif

uint8_t flash_write_data(uint8_t u8Data[], uint8_t uDataLen)
{
  uint8_t * pDataAddr = 0;
  uint8_t nIndex = 0;
  pDataAddr = (uint8_t *)INFO_D_ADDR;
  __disable_interrupt();
  FCTL3 = FWKEY;
  FCTL1 = FWKEY+ERASE;
  for(nIndex = 0; nIndex < uDataLen; nIndex++)
  {
    *(pDataAddr + nIndex) = 0;
  }
  FCTL1 = FWKEY+BLKWRT;
  for(nIndex = 0; nIndex < uDataLen; nIndex++)
  {
    *(pDataAddr + nIndex) = u8Data[nIndex];
  }
  FCTL1 = FWKEY;
  FCTL3 = FWKEY+LOCK;
  __enable_interrupt();
  return 0;
}

uint8_t flash_read_data(uint8_t u8Data[], uint8_t uDataSize)
{
  uint8_t * pDataAddr = (uint8_t *)INFO_D_ADDR;;
  uint8_t nIndex = 0;
  for(nIndex = 0; nIndex < uDataSize; nIndex++)
  {
    u8Data[nIndex] = *(pDataAddr + nIndex);
  }
  return 0;
}