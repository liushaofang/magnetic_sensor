#include "Flash.h"
#include "string.h"

#define INFO_SEGMENT_A_END_ADDR                 0x10ff
#define INFO_SEGMENT_A_START_ADDR               0x10c0
#define INFO_SEGMENT_B_END_ADDR                 0x10bf
#define INFO_SEGMENT_B_START_ADDR               0x1080
#define INFO_SEGMENT_C_END_ADDR                 0x107f
#define INFO_SEGMENT_C_START_ADDR               0x1040
#define INFO_SEGMENT_D_END_ADDR                 0x103f
#define INFO_SEGMENT_D_START_ADDR               0x1000

#define PARAM_MAC_BUF_LEN                       sizeof(Device_Param_Data_Type)
#define PARAM_NETWORK_BUF_LEN                   sizeof(nwkParam_t)                                              //6个字节
#define PARAM_MAGNET_ALG_PARAMS_BUF_LEN         sizeof(Magnet_Alg_Param_Type)
#define PARAM_TRAFFIC_COUNT_BUF_LEN             sizeof(Traffic_Count_Type)

#define PARAM_MAC_ADDR                          (INFO_SEGMENT_B_START_ADDR)                                     //Segment B的低端地址
//2015.04.13:liusf modified for network  
//#define PARAM_NETWORK_ADDR                      (INFO_SEGMENT_B_START_ADDR - PARAM_NETWORK_BUF_LEN)             //Segment C的高端地址
//2015.04.13:liusf modified end
#define PARAM_NETWORK_ADDR                      (INFO_SEGMENT_B_START_ADDR - 20)             //Segment C的高端地址
#define PARAM_TRAFFIC_COUNT_ADDR                (INFO_SEGMENT_C_START_ADDR - PARAM_TRAFFIC_COUNT_BUF_LEN)       //Segment D的高端地址
#define PARAM_MAGNET_ALG_PARAMS_ADDR            (INFO_SEGMENT_D_START_ADDR)                                     //Segment D的地址
#define PARAM_SENSOR_ADDR_LIST_ADDR             (INFO_SEGMENT_D_START_ADDR)                                     //Segment D的地址

uint8_t init_system_flash_rw(void)
{
  WDTCTL = WDTPW + WDTHOLD;
  FCTL2 = FWKEY + FSSEL0 + (FN0 + FN1 + FN2); //FSSEL0表示选择SMCLK作为时钟源，4*FN2+2*FN1+FN0+1=8, SMCLK/8
  return SUCCESS;
}

uint8_t erase_system_flash(uint8_t nSegmentType)
{
  if(nSegmentType & INFO_SEGMENT_TYPE_A)
  {
    return FAILURE;
    /*FCTL3 = FWKEY + LOCKA;
    FCTL1 = FWKEY + ERASE + EEI;
    *(uint8_t *)(INFO_SEGMENT_A_START_ADDR) = 0;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCKA + LOCK;*/
  }
  
  if(nSegmentType & INFO_SEGMENT_TYPE_B)
  {
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE + EEI;
    *(uint8_t *)(INFO_SEGMENT_B_START_ADDR) = 0;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }

  if(nSegmentType & INFO_SEGMENT_TYPE_C)
  {
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE + EEI;
    *(uint8_t *)(INFO_SEGMENT_C_START_ADDR) = 0;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }
  
  if(nSegmentType & INFO_SEGMENT_TYPE_D)
  {
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE + EEI;
    *(uint8_t *)(INFO_SEGMENT_D_START_ADDR) = 0;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }
  return SUCCESS;
}

uint8_t read_system_param(uint16_t nParamType, uint8_t * pParamBuf, uint8_t nParamBufLen)
{
  uint8_t nIndex = 0;
  if(pParamBuf == 0)
    return FAILURE;
  if(nParamType & PARAM_TYPE_MAC)
  {
    if(nParamBufLen < PARAM_MAC_BUF_LEN)
      return FAILURE;
    for(nIndex = 0; nIndex < PARAM_MAC_BUF_LEN;nIndex++)
    {
      pParamBuf[PARAM_MAC_BUF_LEN - nIndex - 1] = *(uint8_t *)(PARAM_MAC_ADDR + nIndex);
    }
  }
  else if(nParamType & PARAM_TYPE_NETWORK_PARAMS)
  {
    if(nParamBufLen < PARAM_NETWORK_BUF_LEN)
      return FAILURE;
    for(nIndex = 0; nIndex < PARAM_NETWORK_BUF_LEN;nIndex++)
    {
      pParamBuf[nIndex] = *(uint8_t *)(PARAM_NETWORK_ADDR + nIndex);
    }
  }
  else if(nParamType & PARAM_TYPE_MAGNET_ALG_PARAMS)
  {
    if(nParamBufLen < PARAM_MAGNET_ALG_PARAMS_BUF_LEN)
      return FAILURE;
    for(nIndex = 0; nIndex < PARAM_MAGNET_ALG_PARAMS_BUF_LEN;nIndex++)
    {
      pParamBuf[nIndex] = *(uint8_t *)(PARAM_MAGNET_ALG_PARAMS_ADDR + nIndex);
    }
  }
  else if(nParamType & PARAM_TYPE_SENSOR_ADDR_LIST)
  {
    if(nParamBufLen > 64)
      return FAILURE;

    for(nIndex = 0; nIndex < nParamBufLen; nIndex++)
    {
      pParamBuf[nIndex] = *(uint8_t *)(PARAM_SENSOR_ADDR_LIST_ADDR + nIndex);
    }
  }
  else if(nParamType & PARAM_TYPE_SENSOR_STATUS)
  {
    //TODO:liusf add, 2014.04.13, 首先清除Flash中的传感器状态，然后写入新的传感器状态
    
  }
  else 
  {
    return FAILURE;
  }
  return SUCCESS;
}

uint8_t write_system_param(uint16_t nParamType, uint8_t * pParamBuf, uint8_t nParamBufLen)
{
  uint8_t nIndex = 0;
  if(nParamType & PARAM_TYPE_MAC)
  {
    if(nParamBufLen < PARAM_MAC_BUF_LEN)
      return FAILURE;
    erase_system_flash(SEGMENT_TYPE_MAC);
    //FCTL3 = FWKEY + LOCKA;
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;
    for(nIndex = 0; nIndex < PARAM_MAC_BUF_LEN; nIndex++)
    {
      *(uint8_t *)(PARAM_MAC_ADDR + nIndex) = pParamBuf[PARAM_MAC_BUF_LEN - nIndex - 1];
    }
    FCTL1 = FWKEY;
    //FCTL3 = FWKEY + LOCKA + LOCK;
    FCTL3 = FWKEY + LOCK;
  }
  else if(nParamType & PARAM_TYPE_NETWORK_PARAMS)
  {
    if(nParamBufLen < PARAM_NETWORK_BUF_LEN)
      return FAILURE;
    erase_system_flash(SEGMENT_TYPE_NETWORK_PARAMS);
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;
    for(nIndex = 0; nIndex < PARAM_NETWORK_BUF_LEN; nIndex++)
    {
      *(uint8_t *)(PARAM_NETWORK_ADDR + nIndex) = pParamBuf[nIndex];
    }
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }
  else if(nParamType & PARAM_TYPE_MAGNET_ALG_PARAMS)
  {
    if(nParamBufLen < PARAM_MAGNET_ALG_PARAMS_BUF_LEN)
      return FAILURE;
    erase_system_flash(SEGMENT_TYPE_MAGNET_ALG_PARAMS);
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;
    for(nIndex = 0; nIndex < PARAM_MAGNET_ALG_PARAMS_BUF_LEN; nIndex++)
    {
      *(uint8_t *)(PARAM_MAGNET_ALG_PARAMS_ADDR + nIndex) = pParamBuf[nIndex];
    }
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }
  else if(nParamType & PARAM_TYPE_SENSOR_ADDR_LIST)
  {
    if(nParamBufLen > INFO_SEGMENT_SIZE)
      return FAILURE;
    erase_system_flash(SEGMENT_TYPE_SENSOR_ADDR_LIST);
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;
    for(nIndex = 0; nIndex < nParamBufLen; nIndex++)
    {
      *(uint8_t *)(PARAM_SENSOR_ADDR_LIST_ADDR + nIndex) = pParamBuf[nIndex];
    }
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
  }
  else if(nParamType & PARAM_TYPE_SENSOR_STATUS)
  {
  }
  else
  {
    return FAILURE;
  }
  return SUCCESS;
}

#define NODE_LIST_COUNT         2

#if 0
uint8_t system_falsh_read_write_test(void)
{
    Device_Param_Data_Type stWrDeviceParamTest;
    Device_Param_Data_Type stRdDeviceParamTest;
    nwkParam_t stWrNwkParamTest;
    nwkParam_t stRdNwkParamTest;
    Magnet_Alg_Param_Type stWrMagnetAlgParamTest;
    Magnet_Alg_Param_Type stRdMagnetAlgParamTest;
    
    addr_t nWrNodeList[NODE_LIST_COUNT] = { 0 };
    addr_t nRdNodeList[NODE_LIST_COUNT] = { 0 };
    
    init_system_flash_rw();
    
    memset(&stWrDeviceParamTest, 0, sizeof(Device_Param_Data_Type));
    stWrDeviceParamTest.chMacAddr[0] = 0x12;
    stWrDeviceParamTest.chMacAddr[1] = 0x34;
    stWrDeviceParamTest.chMacAddr[2] = 0x56;
    DISABLE_WDT;
    write_system_param(PARAM_TYPE_MAC, (uint8_t *)(&stWrDeviceParamTest), sizeof(Device_Param_Data_Type));
    FEED_WDT;
    memset(&stRdDeviceParamTest, 0, sizeof(Device_Param_Data_Type));
    DISABLE_WDT;
    read_system_param(PARAM_TYPE_MAC, (uint8_t *)(&stRdDeviceParamTest), sizeof(Device_Param_Data_Type));
    FEED_WDT;
    if(memcmp((uint8_t *)(&stRdDeviceParamTest), (uint8_t *)(&stWrDeviceParamTest), sizeof(Device_Param_Data_Type)))
      return FAILURE;
    
    memset(&stWrNwkParamTest, 0, sizeof(nwkParam_t));
    stWrNwkParamTest.ip_addr.addr[0] = 0x78;
    stWrNwkParamTest.ip_addr.addr[1] = 0x9A;
    stWrNwkParamTest.channel = 0x10;
    stWrNwkParamTest.tx_power = 0x20;
    stWrNwkParamTest.father_addr.addr[0] = 0xBC;
    stWrNwkParamTest.father_addr.addr[1] = 0xDE;
    DISABLE_WDT;
    write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&stWrNwkParamTest), sizeof(nwkParam_t));
    FEED_WDT;
    memset(&stRdNwkParamTest, 0, sizeof(nwkParam_t));
    DISABLE_WDT;
    read_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&stRdNwkParamTest), sizeof(nwkParam_t));
    FEED_WDT;
    if(memcmp((uint8_t *)(&stRdNwkParamTest), (uint8_t *)(&stWrNwkParamTest), sizeof(nwkParam_t)))
      return FAILURE;
    
    memset(&stWrMagnetAlgParamTest, 0, sizeof(Magnet_Alg_Param_Type));
    stWrMagnetAlgParamTest.nHighThresholdVal = 20;      //120;
    stWrMagnetAlgParamTest.nLowThresholdVal = -20;      //15;
    stWrMagnetAlgParamTest.nIncFluctuationLimit = 2;
    stWrMagnetAlgParamTest.nDecFluctuationLimit = 2;
    DISABLE_WDT;
    write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&stWrMagnetAlgParamTest), sizeof(Magnet_Alg_Param_Type));
    FEED_WDT;
    memset(&stRdMagnetAlgParamTest, 0, sizeof(Magnet_Alg_Param_Type));
    DISABLE_WDT;
    read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&stRdMagnetAlgParamTest), sizeof(Magnet_Alg_Param_Type));
    FEED_WDT;
    if(memcmp((uint8_t *)(&stRdMagnetAlgParamTest), (uint8_t *)(&stWrMagnetAlgParamTest), sizeof(Magnet_Alg_Param_Type)))
      return FAILURE;
    
    nWrNodeList[0].addr[0] = 0x12;
    nWrNodeList[0].addr[1] = 0x34;
    nWrNodeList[1].addr[0] = 0x56;
    nWrNodeList[1].addr[1] = 0x78;
    DISABLE_WDT;
    write_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, (uint8_t *)(&nWrNodeList), sizeof(addr_t) * NODE_LIST_COUNT);
    FEED_WDT;
    memset(&nRdNodeList, 0, sizeof(addr_t) * NODE_LIST_COUNT);
    DISABLE_WDT;
    read_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, (uint8_t *)(&nRdNodeList), sizeof(addr_t) * NODE_LIST_COUNT);
    FEED_WDT;
    if(memcmp((uint8_t *)(&nRdNodeList), (uint8_t *)(&nWrNodeList), sizeof(addr_t) * NODE_LIST_COUNT))
      return FAILURE;
    
    return SUCCESS;
}
#endif
