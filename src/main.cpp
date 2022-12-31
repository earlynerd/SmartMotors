#include <Arduino.h>
// #include <LwipEthernet.h>
// #include "lwIP_adin2111_app.h"
// #include "lwip/timeouts.h"
// #include "lwip/apps/httpd_opts.h"
// #include <FreeRTOS.h>
// #include <stdbool.h>
// #include "adin2111.h"
// #include "boardsupport.h"
// #include <Updater.h>
#include <SPI.h>
#include "Adafruit_TinyUSB.h"
#include "SparkFun_SinglePairEthernet.h"
#include "adin2111.h"
#include "boardsupport.h"

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

volatile byte buffer[2001];
volatile bool newDat = false;
volatile byte sendMacAddr[6];
volatile int rxLen = 0;

uint32_t txPacketCounter = 0, rxPacketCounter = 0;
uint32_t sendInterval;

SinglePairEthernet adin;
byte MAC1[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x50};
byte MAC2[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x4f};
byte pcMac[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x51};

byte *deviceMAC;
byte *destinationMAC;

static void rxCallback(byte *data, int dataLen, byte *senderMac)
{
  if (!newDat)
  {
    memcpy((byte *)buffer, data, dataLen);
    memcpy((byte *)sendMacAddr, senderMac, 6);
    newDat = true;
    rxLen = dataLen;
  }
}

static void linkCallBackFunc(bool linkstat)
{
  // if (linkstat)
  // digitalWrite(RED_LED_PIN, HIGH);
  // else
  // digitalWrite(RED_LED_PIN, LOW);
}

void printStatus(void);
void printPhyState(adi_phy_State_e);
void printMacState(adi_mac_State_e);
void printMacSpiState(adi_mac_SpiState_e);
void printPhyStats(adi_eth_MacStatCounters_t, adi_eth_MacStatCounters_t);

void setup()
{
  pinMode(NET_RESET_PIN, OUTPUT);
  digitalWrite(NET_RESET_PIN, LOW);
  pinMode(NET_IRQ_PIN, INPUT_PULLUP);
  digitalWrite(NET_RESET_PIN, HIGH);

  pinMode(GPI1_PIN, INPUT);
  pinMode(GPI2_PIN, INPUT);
  if (digitalRead(GPI2_PIN))
  {
    deviceMAC = MAC1;
    destinationMAC = MAC2;
    sendInterval = 1000;
  }
  else
  {
    deviceMAC = MAC2;
    destinationMAC = MAC1;
    sendInterval = 1037;
  }

  pinMode(GPO1_PIN, OUTPUT);
  pinMode(GPO2_PIN, OUTPUT);

  Serial.begin(115200);

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
  // digitalWrite(GREEN_LED_PIN, HIGH);
  if (adin.begin(deviceMAC, GREEN_LED_PIN, NET_IRQ_PIN, NET_RESET_PIN, (uint8_t)SPI0_CS))
    Serial.println("ADIN init success");
  else
    Serial.println("ADIN Init fail");
  adin.setRxCallback(rxCallback);
  adin.setLinkCallback(linkCallBackFunc);
  // adin.setRxCallback(NULL);
  adin2111_DeviceId_t id;
  adin.getDeviceId(&id);
  Serial.println(id.phyId, HEX);
  printStatus();
  // error = BSP_InitSystem();
  //  put your setup code here, to run once:
}

uint32_t lastTx = 0;

// adin2111_DeviceHandle_t device = adin.getDeviceHandle();
// LwIP_ADIN2111_t myConn;
// board_t boardDetails;

void loop()
{
  // digitalWrite(GREEN_LED_PIN, LOW);
  /*
  uint32_t       error;
    uint32_t       heartbeatCheckTime = 0;





    //error = BSP_InitSystem();
    //DEBUG_RESULT("BSP_InitSystem", error, 0);

    //BSP_HWReset(true);

    //boardDetails.mac[0] =	deviceMAC[0];
    //boardDetails.mac[1] =	deviceMAC[1];
    //boardDetails.mac[2] =	deviceMAC[2];
    //boardDetails.mac[3] =	deviceMAC[3];
    //boardDetails.mac[4] =	deviceMAC[4];
    //boardDetails.mac[5] =	deviceMAC[5];

    //boardDetails.ip_addr[0] =   192;
    //boardDetails.ip_addr[1] =   168;
    //boardDetails.ip_addr[2] =   7;
    //boardDetails.ip_addr[3] =   100;

    //boardDetails.net_mask[0] =  255;
    //boardDetails.net_mask[1] =  255;
    //boardDetails.net_mask[2] =  255;
    //boardDetails.net_mask[3] =  0;

    //boardDetails.gateway[0] =   192;
    //boardDetails.gateway[1] =   168;
    //boardDetails.gateway[2] =   7;
    //boardDetails.gateway[3] =   1;

    //boardDetails.ip_addr_fixed = IP_DYNAMIC;//IP_FIXED;//

    //error = discoveradin2111(&device);
    //DEBUG_RESULT("Failed to access ADIN2111", error, 0);

    //LwIP_StructInit(&myConn, &device, boardDetails.mac);
    //LwIP_Init(&myConn, &boardDetails);
    //LwIP_ADIN2111LinkInput(&myConn.netif);
    //BSP_delayMs(500);

    //netif_set_link_up(&myConn.netif);
    while(1)
    {
      //uint32_t now  = BSP_SysNow();
      uint32_t now = millis();
      if (now - heartbeatCheckTime >= 250)
      {
        heartbeatCheckTime = now;

        BSP_HeartBeat();

        //sys_check_timeouts();
      }
      //LwIP_ADIN2111LinkInput(&myConn.netif);
    }

  */
  // int numBufsAvailable = adin.getRxAvailable();
  if (millis() - lastTx > sendInterval)
  {
    txPacketCounter++;
    lastTx = millis();
    uint8_t dat1[] = "hello world from 1\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\n";
    uint8_t dat2[] = "hello world from 2\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\nhello world\r\n";
    uint8_t *dat;
    if (digitalRead(GPI2_PIN))
      dat = dat1;
    else
      dat = dat2;
    digitalWrite(RED_LED_PIN, HIGH);
    // adin.setMac(deviceMAC);
    // adin.setDestMac(destinationMAC);
    Serial.print("TX Packet Count: ");
    Serial.println(txPacketCounter);

    adin.sendData(ADIN2111_TX_PORT_AUTO, dat, sizeof(dat1), destinationMAC);
    digitalWrite(RED_LED_PIN, LOW);
    printStatus();
  }

  else if (newDat)
  {
    rxPacketCounter++;
    digitalWrite(BLUE_LED_PIN, HIGH);
    // byte newBuf[2001];
    // memcpy(newBuf, (byte*)buffer, 1524);
    newDat = false;
    // newBuf[1525] = '\0';
    // uint8_t buf;
    // int l = 1;
    // uint8_t addr[6];
    // adin.getRxData(&buf, l, addr);
    // uint32_t cplen = adin.getRxData((byte*)buffer, 2000, (byte*)sendMacAddr);
    // buffer[cplen+1] = '\0';
    buffer[rxLen + 1] = '\0';
    // if (Serial)
    {
      Serial.print("PKT ");
      Serial.print(rxPacketCounter);

      Serial.print(", Recieved ");
      Serial.print(rxLen);
      Serial.println(" bytes: ");
      // Serial.write((char*)buffer);
      for (int i = 0; i < rxLen; i++)
      {
        Serial.print((char)buffer[i]);
        //  buffer[i] = '\0';
      }
      Serial.println();
      // Serial.println((char *)data); //This is ok since we know they are all null terminated strings

      Serial.print("From: ");
      for (int i = 0; i < 6; i++)
      {
        if (sendMacAddr[i] < 0xf)
          Serial.print("0");
        Serial.print(sendMacAddr[i], HEX);
        Serial.print(" ");
      }
      // Serial.print("\t");
      // Serial.print(numBufsAvailable);
      // Serial.print(" before RX, ");
      // Serial.print(adin.getRxAvailable());
      // Serial.println(" after RX");
      Serial.println();
    }
    digitalWrite(BLUE_LED_PIN, LOW);
    // yield();
  }
  // else Serial.println("no data");

  // delay(100);

  // put your main code here, to run repeatedly:
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
  Serial.println();

  adi_eth_MacStatCounters_t port1Stats;
  adi_eth_MacStatCounters_t port2Stats;
  adin2111_GetStatCounters(d, ADIN2111_PORT_1, &port1Stats);
  adin2111_GetStatCounters(d, ADIN2111_PORT_2, &port2Stats);
  printPhyStats(port1Stats, port2Stats);
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
  adi_eth_MacStatCounters_t* stats[2] = {&p1, &p2};
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
    Serial.print(i+1);
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