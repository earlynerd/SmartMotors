/*
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 */
#include <Arduino.h>
#include "lwipopts.h"
#include "SinglePairEthernet.h"
#include "lwip/apps/httpd.h"
#define ADIN2111

// The next three function are static member functions. Static member functions are needed to get a function
// pointer that we can shove into the C function that attaches the interrupt in the driver.
void SinglePairEthernet::linkCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->cbLinkChange(pCBParam, Event, pArg);
    }
}
void SinglePairEthernet::txCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->txCallback(pCBParam, Event, pArg);
    }
    
}
void SinglePairEthernet::rxCallback_C_Compatible(void *pCBParam, uint32_t Event, void *pArg)
{
    
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(pCBParam);
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->rxCallback(pCBParam, Event, pArg);
    }
    
}

err_t SinglePairEthernet::LwIP_ADIN2111LinkOutput_C_Compatible(netif *net, pbuf *buf)
{
    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(net->state);
    //LwIP_ADIN2111_t *eth = reinterpret_cast<LwIP_ADIN2111_t *>();
    //adin2111_DeviceHandle_t device = eth->hDevice;
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->LwIP_ADIN2111LinkOutput(net, buf);
    }
    return ERR_OK;
}

err_t SinglePairEthernet::LwipADIN2111Init_C_Compatible(netif* net)
{

    adin2111_DeviceHandle_t device = reinterpret_cast<adin2111_DeviceHandle_t>(net->state);
    //adin2111_DeviceHandle_t device = eth->hDevice;
    SinglePairEthernet *self = reinterpret_cast<SinglePairEthernet *>(adin2111_GetUserContext(device));
    if (self)
    {
        self->LwipADIN2111Init(net);
    }
    return ERR_OK;
}


err_t SinglePairEthernet::update()
{
    return LwIP_ADIN2111LinkInput(&myConn.netIf);
}

void SinglePairEthernet::txCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    adin2111_DeviceHandle_t device = (adin2111_DeviceHandle_t)pCBParam;   
    txBufAvailable[0] = true;
    adi_eth_BufDesc_t *pRxBufDesc;

    pRxBufDesc = (adi_eth_BufDesc_t *)pArg;
}

void SinglePairEthernet::rxCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    rxFramecount++;
    adin2111_DeviceHandle_t device = (adin2111_DeviceHandle_t)pCBParam;
    adi_eth_BufDesc_t *pRxBufDesc;

    pRxBufDesc = (adi_eth_BufDesc_t *)pArg;

    uint16_t frmLen = pRxBufDesc->trxSize;

    int unicast = ((pRxBufDesc->pBuf[0] & 0x01) == 0);

    LINK_STATS_INC(link.recv);
    MIB2_STATS_NETIF_ADD(device->netIf, ifinoctets, p->tot_len);
    if (unicast)
    {
        MIB2_STATS_NETIF_INC(device->netIf, ifinucastpkts);
    }
    else
    {
        MIB2_STATS_NETIF_INC(device->netIf, ifinnucastpkts);
    }

    writePQ((pQueue_t *)&pQ, pRxBufDesc->pBuf, frmLen);
    /* Since we're not doing anything with the Rx buffer in this example, */
    /* we are re-submitting it to the queue. */
    rxBufDesc[0].pBuf = &rxBuf[0][0];
    rxBufDesc[0].bufSize = MAX_FRAME_BUF_SIZE;
    rxBufDesc[0].cbFunc = rxCallback_C_Compatible;
    adin2111_SubmitRxBuffer(device, pRxBufDesc);
    
    //digitalWrite(25, LOW);
}

uint32_t SinglePairEthernet::adi_phy_GetLinkStatus(uint8_t *status)
{
    uint32_t result = 0;

    *status = (linkState == ADI_ETH_LINK_STATUS_UP) ? 1 : 0;
    return result;
}

uint32_t SinglePairEthernet::sys_now(void)
{
    return millis();
}



void SinglePairEthernet::cbLinkChange(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_eth_LinkStatus_e    linkStatus;

    linkStatus = *(adi_eth_LinkStatus_e *)pArg;
    linkState = linkStatus;
    linkStatusChanged = true;
    (void)linkStatus;
}

bool SinglePairEthernet::begin(uint8_t *mac, uint8_t interrupt, uint8_t reset, uint8_t cs_pin)
{
    myConn.hDevice = &dev;
    //myConn.hDevice = hDevice;
    //myConn.netIf.state = &myConn;
    rxFramecount = 0;
    txFramecount = 0;
    rxBufIndex = 0;
    txBufIndex = 0;
    adi_eth_Result_e result;
    myConn.macAddress = mac;
    
    boardDetails.mac[0] = mac[0];
    boardDetails.mac[1] = mac[1];
    boardDetails.mac[2] = mac[2];
    boardDetails.mac[3] = mac[3];
    boardDetails.mac[4] = mac[4];
    boardDetails.mac[5] = mac[5];

    boardDetails.ip_addr[0] = 192;
    boardDetails.ip_addr[1] = 168;
    boardDetails.ip_addr[2] = 1;
    boardDetails.ip_addr[3] = 100;

    boardDetails.net_mask[0] = 255;
    boardDetails.net_mask[1] = 255;
    boardDetails.net_mask[2] = 255;
    boardDetails.net_mask[3] = 0;

    boardDetails.gateway[0] = 192;
    boardDetails.gateway[1] = 168;
    boardDetails.gateway[2] = 1;
    boardDetails.gateway[3] = 1;

    boardDetails.ip_addr_fixed = IP_FIXED; // IP_FIXED;//

    //myConn.hDevice = hDevice;
    //myConn.macAddress = boardDetails.mac;
    
 
    
    
    BSP_ConfigSystem( interrupt, reset, cs_pin);
    if(BSP_InitSystem())
    {
        return ADI_ETH_DEVICE_UNINITIALIZED;
    }

    BSP_HWReset(true);
    
    //myConn.hDevice->pUserContext = (void *)this;
   
    if(discoveradin2111()) Serial.println("adin init fail");
    result = adin2111_SetUserContext(myConn.hDevice, (void *)this);
     if (result != ADI_ETH_SUCCESS)
    {
        Serial.println("fail setting user context");
    }
    LwIP_StructInit(&myConn, mac);
    LwIP_Init();
    //LwipADIN2111Init(&myConn.netIf);
    
    //LwIP_ADIN2111LinkInput(&myConn.netIf);
    
    BSP_delayMs(500);
    
    

    netif_set_link_up(&myConn.netIf);
    return (result == ADI_ETH_SUCCESS);
}

err_t SinglePairEthernet::LwipADIN2111Init(netif *netIf)
{
   LwIP_ADIN2111_t* eth = &myConn;

   netIf->output = etharp_output;
   netIf->linkoutput = LwIP_ADIN2111LinkOutput_C_Compatible;
   netIf->name[0] = IFNAME0;
   netIf->name[1] = IFNAME1;
   netIf->mtu = ETHERNET_MTU;
   netIf->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netIf->hostname = HOSTNAME;
#endif /* LWIP_NETIF_HOSTNAME */


   netIf->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;
   MIB2_INIT_NETIF(netIf, snmp_ifType_ethernet_csmacd, NETIF_LINK_SPEED_IN_BPS);

   memcpy(netIf->hwaddr, eth->macAddress, 6);
   netIf->hwaddr_len = sizeof(netIf->hwaddr);

   return ERR_OK;
}

adi_eth_Result_e SinglePairEthernet::ADIN2111Init(LwIP_ADIN2111_t* eth)
{
    adi_eth_Result_e        result;
 
    uint8_t  brcstMAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
    adi_mac_AddressRule_t   addrRule;
    addrRule.VALUE16 = 0x0000;
    addrRule.TO_OTHER_PORT = 1;
    addrRule.APPLY2PORT1 = 1;
    addrRule.APPLY2PORT2 = 1;
    addrRule.TO_HOST = 1;

    result = adin2111_AddAddressFilter(myConn.hDevice, brcstMAC, NULL, addrRule);
    DEBUG_RESULT("adin2111_AddAddressFilter", result, ADI_ETH_SUCCESS);
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.println("fail adin2111_AddAddressFilter 1");
    }

    addrRule.VALUE16 = 0x0000;
    addrRule.APPLY2PORT1 = 1;
    addrRule.APPLY2PORT2 = 1;
    addrRule.TO_HOST = 1;

    result = adin2111_AddAddressFilter(myConn.hDevice, eth->macAddress, NULL, addrRule);
    DEBUG_RESULT("adin2111_AddAddressFilter", result, ADI_ETH_SUCCESS);
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.println("fail adin2111_AddAddressFilter 2");
    }
    //adin2111_SetCutThroughMode(myConn.hDevice, true, true, true);
    //adin2111_SetPortForwardMode(myConn.hDevice, ADIN2111_PORT_1, true);
    //adin2111_SetPortForwardMode(myConn.hDevice, ADIN2111_PORT_2, true);
    //adin2111_SetPromiscuousMode(hDevice, ADIN2111_PORT_1, true);
    //adin2111_SetPromiscuousMode(hDevice, ADIN2111_PORT_2, true);
    //addrRule.TO_OTHER_PORT = 1;
    //result = adin2111_AddAddressFilter(*hDevice, brcstMAC, NULL, 0);
    //DEBUG_RESULT("adin2111_AddAddressFilter", result, ADI_ETH_SUCCESS);

    //result = adin2111_AddAddressFilter(hDevice, eth->macAddress, NULL, 0);
   // DEBUG_RESULT("adin2111_AddAddressFilter", result, ADI_ETH_SUCCESS);

    result = adin2111_SyncConfig(myConn.hDevice);
    DEBUG_RESULT("adin2111_SyncConfig", result, ADI_ETH_SUCCESS);
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.println("fail sync config");
    }
    result = adin2111_RegisterCallback(myConn.hDevice, linkCallback_C_Compatible, ADI_MAC_EVT_LINK_CHANGE);
    
    DEBUG_RESULT("adin2111_RegisterCallback (ADI_MAC_EVT_LINK_CHANGE)", result, ADI_ETH_SUCCESS);
    if (result != ADI_ETH_SUCCESS)
    {
        Serial.println("fail register link callback");
    }

        /* Prepare Rx buffers */
    for (uint32_t i = 0; i < 1; i++)
    {
        txBufAvailable[i] = true;
        //rxBufAvailable[i] = false;
        rxBufDesc[i].pBuf = &rxBuf[i][0];
        rxBufDesc[i].bufSize = MAX_FRAME_BUF_SIZE;
        rxBufDesc[i].cbFunc = rxCallback_C_Compatible;

        result = adin2111_SubmitRxBuffer(myConn.hDevice, &rxBufDesc[i]);
    }
    //adin2111_RegisterCallback(myConn.hDevice, rxCallback_C_Compatible, ADI_MAC_EVT_P1_RX_RDY );
    result = adin2111_Enable(myConn.hDevice);
    if (result != ADI_ETH_SUCCESS) Serial.println("Device enable error");
    DEBUG_RESULT("Device enable error", result, ADI_ETH_SUCCESS);

    initQueue((pQueue_t *)&pQ);

  return result;
}


u16_t SinglePairEthernet::ssiHandler_C_Compatible(const char *tag, char *insertBuffer, int insertBufferLen)
{
    return 1;
}


uint32_t SinglePairEthernet::pDataAvailable(pQueue_t *p)
{
    if (p->nWrQ != p->nRdQ)
    {
        return 1;
    }
    return 0;
}

adi_eth_Result_e SinglePairEthernet::LwIP_StructInit(LwIP_ADIN2111_t *eth, uint8_t *macAddress)
{
    //eth->hDevice = &hDevice;
    //eth->macAddress = macAddress;

    return ADI_ETH_SUCCESS;
}

err_t SinglePairEthernet::LwIP_ADIN2111LinkOutput(netif *net, pbuf *buf)
{
    low_level_output(net, buf);
    return ERR_OK;
}

adin2111_DeviceHandle_t SinglePairEthernet::getDeviceHandle()
{
    return myConn.hDevice;
}

void SinglePairEthernet::LwIP_Init( )
{
    //LwIP_ADIN2111_t *eth = &myConn;
    ADIN2111Init(&myConn);
    lwip_init();
    //http_set_ssi_handler(ssiHandler_C_Compatible, NULL, 0);
    //httpd_init();
    ip4_addr_t ipaddr_any;
    ipaddr_any.addr = IPADDR_ANY;
    if (boardDetails.ip_addr_fixed == 1)
    {
      ip4_addr_t ip, mask, gw;
      

      IP4_ADDR(&ip, boardDetails.ip_addr[0], boardDetails.ip_addr[1], boardDetails.ip_addr[2], boardDetails.ip_addr[3]);
      IP4_ADDR(&mask,  boardDetails.net_mask[0], boardDetails.net_mask[1], boardDetails.net_mask[2], boardDetails.net_mask[3]);
      IP4_ADDR(&gw,   boardDetails.gateway[0], boardDetails.gateway[1], boardDetails.gateway[2], boardDetails.gateway[3]);

      netif_add(&myConn.netIf, &ip, &mask, &gw, myConn.hDevice,
      LwipADIN2111Init_C_Compatible, ethernet_input);
      LwipADIN2111Init(&myConn.netIf);
      netif_set_default(&myConn.netIf);
      netif_set_up(&myConn.netIf);
    }
    else
    {
        
      netif_add_noaddr(&myConn.netIf, myConn.hDevice, LwipADIN2111Init_C_Compatible, ethernet_input);
      LwipADIN2111Init(&myConn.netIf);
      netif_set_default(&myConn.netIf);
      netif_set_up(&myConn.netIf);

      dhcp_start(&myConn.netIf);
    }
}

err_t SinglePairEthernet::low_level_output(netif *net, pbuf *p)
{
    //LwIP_ADIN2111_t* eth = &myConn;

    
    txFramecount++;
    struct pbuf *pp;
    uint16_t frameLen = 0;
    int total_len = 0;

    for(pp = p, total_len = 0; pp != NULL; pp = pp->next)
    {
      frameLen =  pp->len ;

      if(frameLen < 2)
      {
        continue;
      }

      memcpy(txBuf[txBufIndex] + total_len  ,(unsigned char*) pp->payload, frameLen);
      total_len += frameLen ;

      if(total_len >= MAX_FRAME_BUF_SIZE)
      {
        return ERR_VAL;
      }
    }

    LINK_STATS_INC(link.xmit);
    MIB2_STATS_NETIF_ADD(net, ifoutoctets, total_len);

    if(total_len < MIN_FRAME_SIZE) // Pad to minimum ETH size
    {
      total_len = MIN_FRAME_SIZE;
    }

    txBufDesc[txBufIndex].pBuf = &txBuf[txBufIndex][0];
    txBufDesc[txBufIndex].trxSize = total_len;
    txBufDesc[txBufIndex].bufSize = MAX_FRAME_BUF_SIZE;
    txBufDesc[txBufIndex].egressCapt = ADI_MAC_EGRESS_CAPTURE_NONE;
    txBufDesc[txBufIndex].cbFunc = txCallback_C_Compatible;

    if ((txBufDesc[txBufIndex].pBuf[0] & 1) != 0)
    {
      /* broadcast or multicast packet*/
      MIB2_STATS_NETIF_INC(net, ifoutnucastpkts);
    }
    else
    {
      /* unicast packet */
      MIB2_STATS_NETIF_INC(net, ifoutucastpkts);
    }

    while(adin2111_SubmitTxBuffer(myConn.hDevice, ADIN2111_TX_PORT_AUTO, &txBufDesc[txBufIndex]) == ADI_ETH_QUEUE_FULL)
    {
      ;;
    }

    if(txBufIndex ++ >= 1 )
    {
      txBufIndex = 0;
    }
   return ERR_OK;
}

err_t SinglePairEthernet::LwIP_ADIN2111LinkInput(netif* net)
{
    if (pDataAvailable((pQueue_t *)&pQ) == 0)
    {
      return ERR_OK;
    }
    else
    {
      pbuf *p = (pbuf *)readPQ((pQueue_t *)&pQ);
      if (p == NULL)
      {
        return ERR_MEM;
      }

      if (net->input(p, net) != ERR_OK)
      {
        LWIP_DEBUGF(NETIF_DEBUG, ("IP input error\r\n"));
        Serial.println("IP input error");
        pbuf_free(p);
        p = NULL;
      }
    }
    return ERR_OK;
}

void SinglePairEthernet::initQueue(SinglePairEthernet::pQueue_t *p)
{
    p->nWrQ = 0;
    p->nRdQ = 0;
}

void SinglePairEthernet::writePQ(SinglePairEthernet::pQueue_t *p, uint8_t *ethFrame, int lenEthFrame)
{
    memcpy(&p->pData[p->nWrQ], ethFrame, lenEthFrame);
    p->lenData[p->nWrQ] = lenEthFrame;
    p->nWrQ = (++(p->nWrQ)%MAX_P_QUEUE);
    //p->nWrQ %= MAX_P_QUEUE;
}

void *SinglePairEthernet::readPQ(SinglePairEthernet::pQueue_t *p)
{
    int ehtFrmLen = p->lenData[p->nRdQ];
    struct pbuf *pb = pbuf_alloc(PBUF_RAW, MAX_FRAME_BUF_SIZE, PBUF_RAM);
    memcpy(((uint8_t *)pb->payload), &p->pData[p->nRdQ], ehtFrmLen);

    p->nRdQ++;
    p->nRdQ %= MAX_P_QUEUE;

    return (void *)pb;
}

uint32_t SinglePairEthernet::discoveradin2111()
{
    adi_eth_Result_e        result;
    uint32_t                error = 1;

    /****** Driver Init *****/
    for (uint32_t i = 0; i < ADIN2111_INIT_ITER; i++)
    {
        result = adin2111_Init(myConn.hDevice, &drvConfig);
        if (result == ADI_ETH_SUCCESS)
        {
            error = 0;
            break;
        }
    }
    if (result != ADI_ETH_SUCCESS) Serial.println("No MACPHY device found");
    DEBUG_RESULT("No MACPHY device found", result, ADI_ETH_SUCCESS);
    
    return error;
}