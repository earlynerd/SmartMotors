/*
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 */
#ifndef __Sparkfun_SinglePairEth__
#define __Sparkfun_SinglePairEth__

#include "lwipopts.h"
#include "adin2111.h"
#include "netif/etharp.h"
#include "lwip/ip_addr.h"
#include "lwip/snmp.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/arch.h"
#include "lwip/apps/httpd.h"
#define MAX_PQ 1
#define MAX_FRAME_BUF_SIZE (MAX_FRAME_SIZE + 4 + 2)

/* Frame size in bytes, applies to each of the transmitted frames. */
/* The value should be between 64 and 1518. */
#define FRAME_SIZE (1518)

/* Number of buffer descriptors to use for both Tx and Rx in this example */
#define BUFF_DESC_COUNT (5)

#define ETHERNET_MTU (1500)

#define IP_FIXED 1
#define IP_DYNAMIC 0
#define MAX_P_QUEUE_SZ 1518
#define MAX_P_QUEUE 5

/* Define name of the network interface. */
#define IFNAME0 (char)'e'
#define IFNAME1 (char)'0'
#define HOSTNAME "ADI_10BASE-T1L_Demo"
#define NETIF_LINK_SPEED_IN_BPS 10000000

#define SPE_MAC_SIZE    6

#define ADIN2111_INIT_ITER  (200)


//const int SPE_NUM_BUFS = 4;
//const int SPE_FRAME_SIZE = 1518;
/* Extra 4 bytes for FCS and 2 bytes for the frame header */
//const int SPE_MAX_BUF_FRAME_SIZE = (SPE_FRAME_SIZE + 4 + 2);/
//const int SPE_MIN_PAYLOAD_SIZE = (46);

//const int SPE_MAC_SIZE = 6;
//const int SPE_FRAME_HEADER_SIZE = (2 * SPE_MAC_SIZE + 2);
//const int SPE_ETHERTYPE_IPV4_B0 = 0x80;
//const int SPE_ETHERTYPE_IPV4_B1 = 0x00;
//extern void http_set_ssi_handler(tSSIHandler pfnSSIHandler,
 //                         const char **ppcTags, int iNumTags);

class SinglePairEthernet 
{
private:
    adin2111_DeviceStruct_t dev;
    //adin2111_DeviceHandle_t hDevice;
    
    uint8_t devMem[ADIN2111_DEVICE_SIZE];
    adin2111_DriverConfig_t drvConfig = {
        .pDevMem    = (void *)devMem,
        .devMemSize = sizeof(devMem),
        .fcsCheckEn = false,
    };

    struct board_t
    {
        uint8_t adin2111Error; // problem with the phy
        uint8_t ip_addr_fixed; // DHCP or fixed IP
        uint8_t ip_addr[4];    /*IP address*/
        uint8_t net_mask[4];   /*network mask*/
        uint8_t gateway[4];    /*IP address*/
        uint8_t mac[6];        /*MAC address*/
    };

    struct pQueue_t
    {
        uint8_t pData[MAX_P_QUEUE][MAX_P_QUEUE_SZ];
        int lenData[MAX_P_QUEUE];
        int32_t nRdQ;
        int32_t nWrQ;
    };

    struct LwIP_ADIN2111_t
    {
        adin2111_DeviceHandle_t hDevice;
        netif netIf;
        uint8_t *macAddress;
    };
    ///////////////////////////////////////

    LwIP_ADIN2111_t myConn;
    HAL_ALIGNED_PRAGMA(4)
    uint8_t txBuf[BUFF_DESC_COUNT][MAX_FRAME_BUF_SIZE] HAL_ALIGNED_ATTRIBUTE(4);
    adi_eth_BufDesc_t txBufDesc[BUFF_DESC_COUNT];
    bool txBufAvailable[BUFF_DESC_COUNT];
    int txBufIndex;

    #define MAX_PQ 1

    HAL_ALIGNED_PRAGMA(4)
    uint8_t rxBuf[BUFF_DESC_COUNT][MAX_FRAME_BUF_SIZE] HAL_ALIGNED_ATTRIBUTE(4);
    adi_eth_BufDesc_t rxBufDesc[BUFF_DESC_COUNT];
    int rxBufIndex;
    
    board_t boardDetails;

    //err_t (*netif_linkoutput_fn) (netif *netif, pbuf *p);

  

    HAL_ALIGNED_PRAGMA(4)
    volatile pQueue_t pQ HAL_ALIGNED_ATTRIBUTE(4);
    err_t low_level_output(netif *, pbuf *);
    void initQueue(pQueue_t *);
    void *readPQ(pQueue_t *);
    void writePQ(pQueue_t *, uint8_t *, int );
    uint32_t pDataAvailable(pQueue_t *);
    uint32_t adi_phy_GetLinkStatus(uint8_t *);
    uint32_t sys_now(void);
    
    
    adi_eth_Result_e LwIP_StructInit(LwIP_ADIN2111_t* eth, uint8_t* macAddress);
    adi_eth_Result_e ADIN2111Init(LwIP_ADIN2111_t* eth);
    err_t LwipADIN2111Init(netif *net);
    bool linkStatusChanged;
    adi_eth_LinkStatus_e linkState;
    uint32_t discoveradin2111();
    void cbLinkChange(void *pCBParam, uint32_t Event, void *pArg);
    err_t LwIP_ADIN2111LinkInput(netif *net);
    ////////////////////////////////////////
    

    // uint8_t rxBuf[SPE_NUM_BUFS][SPE_MAX_BUF_FRAME_SIZE] HAL_ALIGNED_ATTRIBUTE(4);

    //adi_eth_BufDesc_t rxBufDesc[SPE_NUM_BUFS];
    //adi_eth_BufDesc_t txBufDesc[SPE_NUM_BUFS];
    //bool txBufAvailable[SPE_NUM_BUFS];
    //bool rxBufAvailable[SPE_NUM_BUFS];
    //uint32_t txBufIdx;
    //uint32_t rxBufIdx;


    uint8_t macAddr[SPE_MAC_SIZE];
    uint8_t destMacAddr[SPE_MAC_SIZE];
    
    


public:
    err_t update();
    bool begin(uint8_t *mac, uint8_t interrupt, uint8_t reset, uint8_t chip_select);

    //bool sendData(adin2111_TxPort_e port, uint8_t *data, int dataLen, uint8_t *destMac);
    //bool sendData(adin2111_TxPort_e port, uint8_t *data, int dataLen);
    //int getRxData(uint8_t *data, int dataLen, uint8_t *senderMac);
    //int getRxAvailable();
    void LwIP_Init();
    adin2111_DeviceHandle_t getDeviceHandle();
    //void setMac(uint8_t *mac);
    //void getMac(uint8_t *mac);
    //void setDestMac(uint8_t *mac);
    //bool indenticalMacs(uint8_t *mac1, uint8_t *mac2);

    //void setRxCallback(void (*cbFunc)(uint8_t *, int, uint8_t *));
    //void setLinkCallback(void (*cbFunc)(bool));
    //bool getLinkStatus(void);
    //bool getLinkStatus(int);

    // User callbacks
    //void (*userRxCallback)(uint8_t *data, int dataLen, uint8_t *senderMac);
    //void (*userLinkCallback)(bool connected);

    // static functions available to pass to underlying C driver, will regain context and call appropriate member function
    static void txCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg);
    static void rxCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg);
    static void linkCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg);
    static u16_t ssiHandler_C_Compatible(const char* tag, char *insertBuffer, int insertBufferLen);
    static err_t LwIP_ADIN2111LinkOutput_C_Compatible(netif* net, pbuf* buf);
    static err_t LwipADIN2111Init_C_Compatible(netif* net);
    // functions called on successful rx, tx, or link status chage.
    void txCallback(void *pCBParam, uint32_t Event, void *pArg);
    void rxCallback(void *pCBParam, uint32_t Event, void *pArg);
    void linkCallback(void *pCBParam, uint32_t Event, void *pArg);
    err_t LwIP_ADIN2111LinkOutput(netif* net, pbuf* buf);
    //err_t netif_linkoutput_fn(netif*, pbuf* );
    volatile uint32_t rxFramecount;
    volatile uint32_t txFramecount;
};

#endif
