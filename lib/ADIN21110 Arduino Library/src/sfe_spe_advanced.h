/*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions 
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/
#ifndef __Sparkfun_SinglePairEth_Raw__
#define __Sparkfun_SinglePairEth_Raw__

#include <SPI.h>
//#include "adin1110.h"
#include "adin2111.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
//#define SPI_OA_EN
#define SPI_CRC_EN

class sfe_spe_advanced
{
private:
    uint8_t devMem[ADIN2111_DEVICE_SIZE];
    adin2111_DriverConfig_t drvConfig = {
        .pDevMem    = (void *)devMem,
        .devMemSize = sizeof(devMem),
        .fcsCheckEn = false,
    };


public:
    //Device status
    adi_eth_Result_e    begin                   (uint8_t cs_pin = DEFAULT_ETH_SPI_CS_Pin);
    adi_eth_Result_e    begin                   (uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select);
    adi_eth_Result_e    init                    (adin2111_DriverConfig_t *pCfg);
    adi_eth_Result_e    unInit                  (void);
    adi_eth_Result_e    getDeviceId             (adin2111_DeviceId_t *pDevId);
    adi_eth_Result_e    enable                  (void);
    adi_eth_Result_e    disable                 (void);
    adi_eth_Result_e    reset                   (adi_eth_ResetType_e resetType);
    adi_eth_Result_e    syncConfig              (void);
    adi_eth_Result_e    getLinkStatus           (adin2111_Port_e port, adi_eth_LinkStatus_e *linkStatus);
    adi_eth_Result_e    getStatCounters         (adin2111_Port_e port, adi_eth_MacStatCounters_t *stat);
    adi_eth_Result_e    ledEn                   (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    setLoopbackMode         (adin2111_Port_e port, adi_phy_LoopbackMode_e loopbackMode);
    adi_eth_Result_e    setTestMode             (adin2111_Port_e port, adi_phy_TestMode_e testMode);
    adi_eth_Result_e    addAddressFilter        (uint8_t *macAddr, uint8_t *macAddrMask, adi_mac_AddressRule_t addrRule);
    adi_eth_Result_e    clearAddressFilter      (uint32_t addrIndex);
    adi_eth_Result_e    submitTxBuffer          (adin2111_TxPort_e port, adi_eth_BufDesc_t *pBufDesc);
    adi_eth_Result_e    submitRxBuffer          (adi_eth_BufDesc_t *pBufDesc);
    #if defined(ADI_MAC_ENABLE_RX_QUEUE_HI_PRIO)
    adi_eth_Result_e    submitRxBufferHp        (adi_eth_BufDesc_t *pBufDesc);
    #endif
    adi_eth_Result_e    setPromiscuousMode      (adin2111_Port_e port, bool bFlag);
    adi_eth_Result_e    getPromiscuousMode      (adin2111_Port_e port, bool *pFlag);
    #if defined(SPI_OA_EN)
    adi_eth_Result_e    setChunkSize            (adi_mac_OaCps_e cps);
    adi_eth_Result_e    getChunkSize            (adi_mac_OaCps_e *pCps);
    #endif
    adi_eth_Result_e    setCutThroughMode       (bool txcte, bool rxcte, bool p2pcte);
    adi_eth_Result_e    getCutThroughMode       (bool *pTxcte, bool *pRxcte, bool *p2pcte);
    adi_eth_Result_e    setFifoSizes            (adi_mac_FifoSizes_t fifoSizes);
    adi_eth_Result_e    getFifoSizes            (adi_mac_FifoSizes_t *pFifoSizes);
    adi_eth_Result_e    clearFifos              (adi_mac_FifoClrMode_e clearMode);
    adi_eth_Result_e    tsEnable                (adi_mac_TsFormat_e format);
    adi_eth_Result_e    tsClear                 (void);
    adi_eth_Result_e    tsTimerStart            (adi_mac_TsTimerConfig_t *pTimerConfig);
    adi_eth_Result_e    tsTimerStop             (void);
    adi_eth_Result_e    tsSetTimerAbsolute      (uint32_t seconds, uint32_t nanoseconds);
    adi_eth_Result_e    tsSyncClock             (int64_t tError, uint64_t referenceTimeNsDiff, uint64_t localTimeNsDiff);
    //adi_eth_Result_e    tsGetExtCaptTimestamp   (adi_mac_TsTimespec_t *pCapturedTimespec);
    adi_eth_Result_e    tsGetEgressTimestamp    (adi_mac_EgressCapture_e egressReg, adi_mac_TsTimespec_t *pCapturedTimespec);
    adi_eth_Result_e    tsConvert               (uint32_t timestampLowWord, uint32_t timestampHighWord, adi_mac_TsFormat_e format, adi_mac_TsTimespec_t *pTimespec);
    int64_t             tsSubtract              (adi_mac_TsTimespec_t *pTsA, adi_mac_TsTimespec_t *pTsB);
    adi_eth_Result_e    registerCallback        (adi_eth_Callback_t cbFunc, adi_mac_InterruptEvt_e cbEvent);
    adi_eth_Result_e    setUserContext          (void *pContext);
    void *              getUserContext          (void);
    adi_eth_Result_e    writeRegister           (uint16_t regAddr, uint32_t regData);
    adi_eth_Result_e    readRegister            (uint16_t regAddr, uint32_t *regData);
    adi_eth_Result_e    phyWrite                (adin2111_Port_e port, uint32_t regAddr, uint16_t regData);
    adi_eth_Result_e    phyRead                 (adin2111_Port_e port, uint32_t regAddr, uint16_t *regData);
    adi_eth_Result_e    getMseLinkQuality       (adin2111_Port_e port, adi_phy_MseLinkQuality_t *mseLinkQuality);
    adi_eth_Result_e    frameGenEn              (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    frameGenSetMode         (adin2111_Port_e port, adi_phy_FrameGenMode_e mode);
    adi_eth_Result_e    frameGenSetFrameCnt     (adin2111_Port_e port, uint32_t frameCnt);
    adi_eth_Result_e    frameGenSetFramePayload (adin2111_Port_e port, adi_phy_FrameGenPayload_e payload);
    adi_eth_Result_e    frameGenSetFrameLen     (adin2111_Port_e port, uint16_t frameLen);
    adi_eth_Result_e    frameGenSetIfgLen       (adin2111_Port_e port, uint16_t ifgLen);
    adi_eth_Result_e    frameGenRestart         (adin2111_Port_e port);
    adi_eth_Result_e    frameGenDone            (adin2111_Port_e port, bool *fgDone);
    adi_eth_Result_e    frameChkEn              (adin2111_Port_e port, bool enable);
    adi_eth_Result_e    frameChkSourceSelect    (adin2111_Port_e port, adi_phy_FrameChkSource_e source);
    adi_eth_Result_e    frameChkReadFrameCnt    (adin2111_Port_e port, uint32_t *cnt);
    adi_eth_Result_e    frameChkReadRxErrCnt    (adin2111_Port_e port, uint16_t *cnt);
    adi_eth_Result_e    frameChkReadErrorCnt    (adin2111_Port_e port, adi_phy_FrameChkErrorCounters_t *cnt);
    adin2111_DeviceHandle_t getDeviceHandle(void);
    adin2111_DeviceStruct_t dev;
    adin2111_DeviceHandle_t hDevice = &dev;
};

#endif
