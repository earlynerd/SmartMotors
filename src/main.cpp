#include <Arduino.h>
// #include "adin2111.h"
// #include "lwIP_adin2111_app.h"
// #include "lwip/timeouts.h"
#include <SPI.h>
#include "LwipEthernet.h"
#include "lwipopts.h"
#include "netif/etharp.h"
#include "lwip/ip_addr.h"
#include "lwip/snmp.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/arch.h"
#include "lwip/apps/httpd.h"
#include "Adafruit_TinyUSB.h"
#include "SinglePairEthernet.h"

// #include "boardsupport.h"

#define SPI0_MISO 16
#define SPI0_MOSI 19
#define SPI0_SCK 18
#define SPI0_CS 17

#define NET_IRQ_PIN 6
#define NET_RESET_PIN 7

#define SPI1_MISO 12
#define SPI1_MOSI 11
#define SPI1_SCK 10
#define SPI1_CS 23

#define DRIVE_EN_PIN 22

#define RED_LED_PIN 26
#define GREEN_LED_PIN 27
#define BLUE_LED_PIN 25

#define SDA0_PIN 20
#define SCL0_PIN 21

#define SDA1_PIN 2
#define SCL1_PIN 3

#define UART0_TX 0
#define UART0_RX 1

#define GPI1_PIN 5
#define GPI2_PIN 4

#define GPO1_PIN 13
#define GPO2_PIN 14

#define T0_PIN 28
#define T1_PIN 29

SinglePairEthernet adin;

byte deviceMAC[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x50};
uint8_t *mac = deviceMAC;
void printStatus(void);
void printPhyState(adi_phy_State_e);
void printMacState(adi_mac_State_e);
void printMacSpiState(adi_mac_SpiState_e);
void printPhyStats(adi_eth_MacStatCounters_t, adi_eth_MacStatCounters_t);
void printOAerrorStats(adi_mac_OaErrorStats_t e);
void printIP(ip_addr_t addr);

void setup()
{
  Serial.begin(115200);
  pinMode(NET_RESET_PIN, OUTPUT);
  digitalWrite(NET_RESET_PIN, LOW);
  pinMode(NET_IRQ_PIN, INPUT_PULLUP);
  digitalWrite(NET_RESET_PIN, HIGH);

  pinMode(GPI1_PIN, INPUT);
  pinMode(GPI2_PIN, INPUT);

  pinMode(GPO1_PIN, OUTPUT);
  pinMode(GPO2_PIN, OUTPUT);

  SPI.begin();
  delay(3000);
  for (int i = 0; i < 6; i++)
  {
    if (deviceMAC[i] < 0xF)
      Serial.print("0");
    Serial.print(deviceMAC[i], HEX);
    if (i < 5)
      Serial.print(" ");
    else
      Serial.println();
  }
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
}

uint32_t lastTx = 0;
uint32_t statusInterval = 500;

void loop()
{
  if (adin.begin(mac, NET_IRQ_PIN, NET_RESET_PIN, (uint8_t)SPI0_CS))
    Serial.println("ADIN init success");
  else
  {
    Serial.println("ADIN Init fail");
    for (;;)
      yield();
  }

  for (;;)
  {

    if (millis() - lastTx > statusInterval)
    {
      digitalWrite(GREEN_LED_PIN, HIGH);
      lastTx = millis();
      printStatus();
      digitalWrite(GREEN_LED_PIN, LOW);
    }
    err_t error = adin.update();
    if (error == ERR_MEM)
      Serial.println("Error -  memory");
  }
}

void printStatus()
{
  adin2111_DeviceHandle_t d = adin.getDeviceHandle();
  adi_phy_Device_t *pPhyP1 = d->pPhyDevice[ADIN2111_PORT_1];
  adi_phy_Device_t *pPhyP2 = d->pPhyDevice[ADIN2111_PORT_2];
  adi_mac_Device_t *pMacTemp = d->pMacDevice;
  
  Serial.print("ADIN2111 port 1 ");
  if (d->portEnabled[ADIN2111_PORT_1])
    Serial.println("enabled");
  else
    Serial.println("NOT enabled");
  Serial.print("ADIN2111 port 2 ");
  if (d->portEnabled[ADIN2111_PORT_2])
    Serial.println("enabled");
  else
    Serial.println("NOT enabled");

  Serial.print("MAC state: ");
  printMacState(pMacTemp->state);

  Serial.print("MAC SPI state: ");
  printMacSpiState(pMacTemp->spiState);

  adi_mac_StatusRegisters_t macStatus = pMacTemp->statusRegisters;
  Serial.print("Port 1 mac status: ");
  Serial.println(macStatus.p1Status, HEX);
  Serial.print("Port 2 mac status: ");
  Serial.println(macStatus.p2Status, HEX);
  Serial.print("mac status0: ");
  Serial.println(macStatus.status0, HEX);
  Serial.print("mac status1: ");
  Serial.println(macStatus.status1, HEX);
  Serial.print("MAC IRQ mask: ");
  Serial.print(pMacTemp->irqMask1, HEX);
  Serial.println(pMacTemp->irqMask0, HEX);

  adi_phy_State_e p1State = pPhyP1->state;
  adi_phy_State_e p2State = pPhyP2->state;
  Serial.print("PHY 1 state: ");
  printPhyState(p1State);
  Serial.print("PHY 2 state: ");
  printPhyState(p2State);

  Serial.print("PHY 1 IRQ mask: ");
  Serial.println(pPhyP1->irqMask, HEX);
  Serial.print("PHY 2 IRQ mask: ");
  Serial.println(pPhyP2->irqMask, HEX);

  if(pPhyP1->irqPending) Serial.println("PHY 1 IRQ pending");
  if(pPhyP2->irqPending) Serial.println("PHY 2 IRQ pending");

  Serial.print("Phy port 1 link status: ");
  if (pPhyP1->linkStatus == adi_phy_LinkStatus_e::ADI_PHY_LINK_STATUS_UP)
    Serial.println("UP");
  else
    Serial.println("DOWN");
  Serial.print("Phy port 2 link status: ");
  if (pPhyP2->linkStatus == adi_phy_LinkStatus_e::ADI_PHY_LINK_STATUS_UP)
    Serial.println("UP");
  else
    Serial.println("DOWN");
  Serial.print("Phy 1 link dropped counter: ");
  Serial.println(pPhyP1->stats.linkDropped);
  Serial.print("Phy 2 link dropped counter: ");
  Serial.println(pPhyP2->stats.linkDropped);

  Serial.print("OA Tx frame Queue index: ");
  Serial.println(d->pMacDevice->oaTxCurBufIdx);
  Serial.print("OA Rx frame Queue index: ");
  Serial.println(d->pMacDevice->oaRxCurBufIdx);

  adi_mac_OaErrorStats_t e = d->pMacDevice->oaErrorStats;
  printOAerrorStats(e);

  adi_eth_MacStatCounters_t port1Stats;
  adi_eth_MacStatCounters_t port2Stats;
  adin2111_GetStatCounters(d, ADIN2111_PORT_1, &port1Stats);
  adin2111_GetStatCounters(d, ADIN2111_PORT_2, &port2Stats);
  printPhyStats(port1Stats, port2Stats);
  Serial.println();

  uint8_t count = 1;
  netif *net = netif_get_by_index(1);
  while (net != NULL)
  {
    Serial.print("netif ");
    Serial.print(count);
    Serial.print(": ");
    Serial.print(net->name[0]);
    Serial.print(net->name[1]);
    Serial.println();
    

    // net = netif_get_by_index(1);
    ip_addr_t ip = net->ip_addr;
    Serial.print("IP: ");
    printIP(ip);
    ip = net->gw;
    Serial.print("Gateway: ");
    printIP(ip);
    ip = net->netmask;
    Serial.print("Netmask: ");
    printIP(ip);

    Serial.print("HW address: ");
    for (int i = 0; i < 6; i++)
    {
      if (net->hwaddr[i] < 0xf)
        Serial.print("0");
      Serial.print(net->hwaddr[i], HEX);
      if (i < 5)
        Serial.print(":");
      else
        Serial.println();
    }
    // memcpy(net->hwaddr, mac, 6);

    if (netif_is_up(net))
      Serial.println("netif is UP");
    else
    {
      Serial.println("netif is DOWN");
      // netif_set_up(net);
    }
    if (netif_is_link_up(net))
      Serial.println("netif Link is UP");
    else
      Serial.println("netif Link is DOWN");
    //net = netif_get_by_index(count);
    net = net->next;
    count++;
  }
  Serial.println();
  Serial.print("rx count: ");
  Serial.print(adin.rxFramecount);
  Serial.print(", tx count: ");
  Serial.println(adin.txFramecount);
  if(digitalRead(NET_IRQ_PIN)) Serial.println("IRQ pin HIGH");
  else Serial.println("IRQ pin LOW");
}

/*

RX_FRM;         !< Rx frame count.
RX_BCAST;       !< Rx unicast frame count.
RX_MCAST;       !< Rx multicast frame count.
RX_UCAST;       !< Rx broadcast frame count.
RX_CRC_ERR;     !< Rx CRC errored frame count.
RX_ALGN_ERR;    !< Rx alignment error count.
RX_LS_ERR;      !< Rx long/Short error count.
RX_PHY_ERR;     !< Rx PHY error (MII RX_ER).
TX_FRM;         !< Tx frame count.
TX_BCAST;       !< Tx unicast frame count.
TX_MCAST;       !< Tx multicast frame count.
TX_UCAST;       !< Tx broadcast frame count.
RX_DROP_FULL;   !< Rx dropped due to FIFO full count.
RX_DROP_FILT;   !< Rx dropped due to filtering count.
*/
void printPhyStats(adi_eth_MacStatCounters_t p1, adi_eth_MacStatCounters_t p2)
{
  adi_eth_MacStatCounters_t *stats[2] = {&p1, &p2};
  const char names[15][16] =
      {
          {"Port\t\0"},
          {"RX_FRM\t\0"},
          {"RX_BCAST\t\0"},
          {"RX_MCAST\t\0"},
          {"RX_UCAST\t\0"},
          {"RX_CRC_ERR\t\0"},
          {"RX_ALGN_ERR\t\0"},
          {"RX_LS_ERR\t\0"},
          {"RX_PHY_ERR\t\0"},
          {"TX_FRM\t\0"},
          {"TX_BCAST\t\0"},
          {"TX_MCAST\t\0"},
          {"TX_UCAST\t\0"},
          {"RX_DROP_FULL\t\0"},
          {"RX_DROP_FILT\t\0"}};

  for (int i = 0; i < 15; i++)
  {
    Serial.print(names[i]);
  }
  Serial.println();

  for (int i = 0; i < 2; i++)
  {
    Serial.print("Port ");
    Serial.print(i + 1);
    Serial.print("\t");
    Serial.print(stats[i]->RX_FRM_CNT);
    Serial.print("\t");
    Serial.print(stats[i]->RX_BCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_MCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_UCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_CRC_ERR_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_ALGN_ERR_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_LS_ERR_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_PHY_ERR_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->TX_FRM_CNT);
    Serial.print("\t");
    Serial.print(stats[i]->TX_BCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->TX_MCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->TX_UCAST_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_DROP_FULL_CNT);
    Serial.print("\t\t");
    Serial.print(stats[i]->RX_DROP_FILT_CNT);
    Serial.println();
  }
}

void printPhyState(adi_phy_State_e p)
{
  switch (p)
  {
  case adi_phy_State_e::ADI_PHY_STATE_DIAGNOSTIC:
    Serial.println("ADI_PHY_STATE_DIAGNOSTIC");
    break;
  case adi_phy_State_e::ADI_PHY_STATE_ERROR:
    Serial.println("ADI_PHY_STATE_ERROR");
    break;
  case adi_phy_State_e::ADI_PHY_STATE_HW_RESET:
    Serial.println("ADI_PHY_STATE_HW_RESET");
    break;
  case adi_phy_State_e::ADI_PHY_STATE_OPERATION:
    Serial.println("ADI_PHY_STATE_OPERATION");
    break;
  case adi_phy_State_e::ADI_PHY_STATE_SOFTWARE_POWERDOWN:
    Serial.println("ADI_PHY_STATE_SOFTWARE_POWERDOWN");
    break;
  case adi_phy_State_e::ADI_PHY_STATE_UNINITIALIZED:
    Serial.println("ADI_PHY_STATE_UNINITIALIZED");
    break;
  }
}

void printMacState(adi_mac_State_e m)
{
  switch (m)
  {
  case adi_mac_State_e::ADI_MAC_STATE_UNINITIALIZED:
    Serial.println("ADI_MAC_STATE_UNINITIALIZED");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_INITIALIZED:
    Serial.println("ADI_MAC_STATE_INITIALIZED");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_READY:
    Serial.println("ADI_MAC_STATE_READY");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_TX_FRAME:
    Serial.println("ADI_MAC_STATE_TX_FRAME");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_RX_FRAME:
    Serial.println("ADI_MAC_STATE_RX_FRAME");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_CONTROL_START:
    Serial.println("ADI_MAC_STATE_CONTROL_START");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_CONTROL_END:
    Serial.println("ADI_MAC_STATE_CONTROL_END");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_DATA_START:
    Serial.println("ADI_MAC_STATE_DATA_START");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_DATA_END:
    Serial.println("ADI_MAC_STATE_DATA_END");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_IRQ_START:
    Serial.println("ADI_MAC_STATE_IRQ_START");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_DATA_READ_STATUS:
    Serial.println("ADI_MAC_STATE_DATA_READ_STATUS");
    break;
  case adi_mac_State_e::ADI_MAC_STATE_DATA_READ_PHY_REGISTER:
    Serial.println("ADI_MAC_STATE_DATA_READ_PHY_REGISTER");
    break;
  }
}

void printMacSpiState(adi_mac_SpiState_e s)
{
  switch (s)
  {
  case adi_mac_SpiState_e::ADI_MAC_SPI_STATE_READY:
    Serial.println("ADI_MAC_SPI_STATE_READY");
    break;
  case adi_mac_SpiState_e::ADI_MAC_SPI_STATE_RX:
    Serial.println("ADI_MAC_SPI_STATE_RX");
    break;
  case adi_mac_SpiState_e::ADI_MAC_SPI_STATE_RX_FRAME:
    Serial.println("ADI_MAC_SPI_STATE_RX_FRAME");
    break;
  case adi_mac_SpiState_e::ADI_MAC_SPI_STATE_TX:
    Serial.println("ADI_MAC_SPI_STATE_TX");
    break;
  case adi_mac_SpiState_e::ADI_MAC_SPI_STATE_TX_FRAME:
    Serial.println("ADI_MAC_SPI_STATE_TX_FRAME");
    break;
  }
}

void printOAerrorStats(adi_mac_OaErrorStats_t e)
{
  Serial.print("fdCount: ");
  Serial.print(e.fdCount);
  Serial.print(", ");
  Serial.print("ftrParityErrorCount: ");
  Serial.print(e.ftrParityErrorCount);
  Serial.print(", ");
  Serial.print("hdrParityErrorCount: ");
  Serial.print(e.hdrParityErrorCount);
  Serial.print(", ");
  Serial.print("invalidEvCount: ");
  Serial.print(e.invalidEvCount);
  Serial.print(", ");
  Serial.print("invalidSvCount: ");
  Serial.print(e.invalidSvCount);
  Serial.print(", ");
  Serial.print("syncErrorCount: ");
  Serial.print(e.syncErrorCount);
  Serial.println();
}

void printIP(ip_addr_t a)
{

  uint32_t b = a.addr;
  for (int i = 0; i < 4; i++)
  {
    Serial.print((b >> (8 * i)) & 0xff);
    if (i < 3)
      Serial.print(".");
    else
      Serial.println();
  }
}