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
#include "Adafruit_TinyUSB.h"
#include "SparkFun_SinglePairEthernet.h"
#include "usb_network.h"
#include <SPI.h>

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

volatile byte buffer[2001];
volatile bool newDat = false;
volatile byte sendMacAddr[6];
volatile int rxLen = 0;

SinglePairEthernet adin;
byte deviceMAC[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x50};
byte destinationMAC[6] = {0x08, 0x3A, 0x88, 0x5C, 0x18, 0x4F};

static void rxCallback(byte *data, int dataLen, byte *senderMac)
{
  if(!newDat)
  {
    memcpy((byte*)buffer, data, sizeof(buffer));
    memcpy((byte*)sendMacAddr, senderMac, 6);
    newDat = true;
    rxLen = dataLen;
  }
}

static void linkCallBackFunc(bool linkstat)
{
  if(linkstat) digitalWrite(RED_LED_PIN, HIGH);
  else digitalWrite(RED_LED_PIN, LOW);

}

void setup()
{
  pinMode(NET_RESET_PIN, OUTPUT);
  digitalWrite(NET_RESET_PIN, LOW);
  pinMode(NET_IRQ_PIN, INPUT_PULLUP);
  digitalWrite(NET_RESET_PIN, HIGH);
  Serial.begin(115200);
  SPI.begin();
  delay(5000);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, HIGH);
  if (adin.begin(deviceMAC, GREEN_LED_PIN, NET_IRQ_PIN, NET_RESET_PIN, (uint8_t)SPI0_CS))
    Serial.println("ADIN init success");
  else
    Serial.println("ADIN Init fail");
  adin.setRxCallback(rxCallback);
  adin.setLinkCallback(linkCallBackFunc);
  //adin.setRxCallback(NULL);
  adin2111_DeviceId_t id;
  adin.getDeviceId(&id);
  Serial.println(id.phyId, HEX);
  adin.setPromiscuousMode(adin2111_Port_e::ADIN2111_PORT_1, true);
  adin.setPromiscuousMode(adin2111_Port_e::ADIN2111_PORT_2, true);

  // error = BSP_InitSystem();
  //  put your setup code here, to run once:
}

uint32_t lastTx = 0;

// adin2111_DeviceHandle_t device = adin.getDeviceHandle();
// LwIP_ADIN2111_t myConn;
// board_t boardDetails;

void loop()
{
  digitalWrite(GREEN_LED_PIN, LOW);
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
  if (millis() - lastTx > 100)
  {
    lastTx = millis();
    uint8_t dat[] = "hello world\r\n";
    digitalWrite(GREEN_LED_PIN, HIGH);
    //adin.setMac(deviceMAC);
    //adin.setDestMac(destinationMAC);
    adin.sendData(ADIN2111_TX_PORT_FLOOD, dat, 14, destinationMAC);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  int numBufsAvailable = adin.getRxAvailable();
  if (newDat)
  {
    digitalWrite(BLUE_LED_PIN, HIGH);
    //byte newBuf[2001];
    //memcpy(newBuf, (byte*)buffer, 1524);
    newDat = false;
    //newBuf[1525] = '\0';
    //uint8_t buf;
    //int l = 1;
    //uint8_t addr[6];
    //adin.getRxData(&buf, l, addr);
    //uint32_t cplen = adin.getRxData((byte*)buffer, 2000, (byte*)sendMacAddr);
    //buffer[cplen+1] = '\0';
    buffer[rxLen+1] = '\0';
    //if (Serial)
    {
      Serial.print("Recieved ");
      Serial.print(rxLen);
      Serial.println(" bytes: ");
      //Serial.write((char*)buffer);
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
      Serial.print("\t");
      Serial.print(numBufsAvailable);
      Serial.print(" before RX, ");
      Serial.print(adin.getRxAvailable());
      Serial.println(" after RX");
      Serial.println();
    }
    digitalWrite(BLUE_LED_PIN, LOW);
    //yield();
  }
  // else Serial.println("no data");

  // delay(100);

  // put your main code here, to run repeatedly:
}