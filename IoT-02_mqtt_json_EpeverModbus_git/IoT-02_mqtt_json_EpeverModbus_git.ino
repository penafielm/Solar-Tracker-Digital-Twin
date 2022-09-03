
// https://github.com/penafielm/Solar-Tracker-Digital-Twin
// 
//
// Code written by Ignacio Peñafiel Miguel and mentorship by Jordi Binefa. 20220901
// https://github.com/jordibinefa/IoT-02/tree/master/codes
// things.cat
//

#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
// #ifdef XIP_U14

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#include "IoT-02_pinout.h"
#include "IoT-02_common.h"
#include <WiFi.h>
#include "IoT-02_wifiMng.h"
#include "IoT-02_mqttCredentials.h"
#include "IoT-02_mqttTopics.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <Arduino_JSON.h>

#include "IoT-02_bme280.h"
Adafruit_BME280 bme;

#include "IoT-02_oled.h"
SSD1306  display(0x3c, I2C_SDA, I2C_SCL);

#include "Modbus.h"
#include "PeticioModbus.h"
#include "RecieveModbus.h"
//HardwareSerial modbusData(2);
#define BAUD_MPPT 115200
#define MAX_STR_LEN 50
#define TIMEOUT_ms  300


/* create an instance of WiFiClientSecure */
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);
#define RECONNECTING_INTERVAL 30000
#include <PubSubClient.h>
#define MQTT_MAX_PACKET_SIZE 2048

#define MAC_SIZE 15
char sMac[MAC_SIZE];
char sModbus[16];

JSONVar json_IoT;
#include <stdio.h>

#define BATTERY_OVER_VOLTAGE                0x0001 /* 0000 0000 0000 0001 */
#define BATTERY_UNDER_VOLTAGE               0x0002 /* 0000 0000 0000 0010 */
#define BATTERY_OVER_DISCHARGE              0x0003 /* 0000 0000 0000 0011 */
#define BATTERY_FAULT                       0x0004 /* 0000 0000 0000 0100 */

#define BATTERY_OVER_TEMP                   0x0010 /* 0000 0000 0001 0000 */
#define BATTERY_LOW_TEMP                    0x0020 /* 000 00000 0010 0000 */

#define BATTERY_ABNORMAL_INNER_RESISTANCE   0x0100 /* 0000 0001 0000 0000 */
#define BATTERY_WRONG_RATED_VOLTAGE         0x1000 /* 0001 0000 0000 0000 */


#define CHARGING_RUNNING                    0x0001 /* 0000 0000 0000 0001 */
#define CHARGING_FAULT                      0x0002 /* 0000 0000 0000 0010 */
#define CHARGING_FLOAT                      0x0004 /* 0000 0000 0000 0100 */
#define CHARGING_BOOST                      0x0008 /* 0000 0000 0000 1000 */
#define CHARGING_EQUALIZATION               0x000C /* 0000 0000 0000 1100 */

#define PV_INPUT_SHORTCIRCUIT               0x0010 /* 0000 0000 0001 0000 */
#define PV_DISEQUILIBRIUM                   0x0040 /* 000 00000 0100 0000 */

#define MPPT_LOAD_MOSFET_SHORTCIRCUIT       0x0080 /* 0000 0000 1000 0000 */
#define MPPT_LOAD_SHORTCIRCUIT              0x0100 /* 0000 0001 0000 0000 */
#define MPPT_LOAD_OVERCURRENT               0x0200 /* 0000 0010 0000 0000 */
#define MPPT_INPUT_OVERCURRENT              0x0400 /* 0000 0100 0000 0000 */
#define MPPT_REVERSE_MOSFET_SHORTCIRCUIT    0x0800 /* 0000 1000 0000 0000 */

#define MPPT_REVERSE_MOSFET_OPENCIRCUIT     0x1000 /* 0001 0000 0000 0000 */
#define MPPT_CHARGING_MOSFET_SHORTCIRCUIT   0x2000 /* 0010 0000 0000 0000 */
#define MPPT_NO_INPUT_POWER                 0x4000 /* 0100 0000 0000 0000 */
#define MPPT_HIGH_INPUT_VOLTAGE             0x8000 /* 1000 0000 0000 0000 */
#define MPPT_INPUT_VOLTAGE_ERROR            0xC000 /* 1100 0000 0000 0000 */

// 0xA500 & 0x0F00 = 0x0500 /*1010 0101 0000 0000 & 0000 1111 0000 0000 = 0000 0101 0000 0000 */
// 0xA500 & 0xF000 = 0xA000
// 0xA500 & 0xFF00 = 0xA500
// 0xA500 & 0x8000 = 0x8000 <- True
// 0xA500 & 0x4000 = 0x0000 <- False
// 0xA500 & 0x2000 = 0x2000 <- True
// 0xA500 & 0x1000 = 0x0000 <- False
// 0xA500 & 0x0800 = 0x0000 <- False
// 0xA500 & 0x0400 = 0x0400 <- True


String vAnalizaBattery(unsigned int ui,char* sz){
    unsigned int uiLBLN = ui & 0x000F;
    unsigned int uiLBHN = ui & 0x00F0;
    unsigned int uiSBLN = ui & 0x0F00;
    unsigned int uiSBHN = ui & 0xF000;
    String szz;
     
    //printf("%s:\n",sz);
    switch (uiLBLN) {
      case BATTERY_OVER_DISCHARGE:
        //printf("BATTERY_Over discharge\n");
        szz= String("NO_CH").c_str();
        break;
      case BATTERY_OVER_VOLTAGE:
        //printf("BATTERY_Over voltage\n");
        szz= String("HIGH_V").c_str();
        break;   
      case BATTERY_UNDER_VOLTAGE:
        //printf("BATTERY_Under voltage\n");
        szz= String("LOW_V").c_str();
        break;
      case BATTERY_FAULT:
        //printf("BATTERY_FAULT\n");
        szz= String("FAIL").c_str();
        break;
      default:
        //printf("BATTERY_NORMAL\n");
        szz= String("NORM").c_str();
        break;
           
    }
    switch (uiLBHN) {
      case BATTERY_ABNORMAL_INNER_RESISTANCE:
        //printf("BATTERY_ABNORMAL_INNER_RESISTANCE\n");
        szz= String(szz + "/" + "ABNORM").c_str();
        break;  
      default:
        //printf("BATTERY_NORMAL resistance\n");
        szz= String(szz + "/" +  "NORM").c_str();
        break;
           
    }

     switch (uiSBLN) {
      case BATTERY_WRONG_RATED_VOLTAGE:
        //printf("BATTERY_WRONG_RATED_VOLTAGE\n");
        szz= String(szz + "/" + "FAIL_V").c_str();
        break;
      default:
        //printf("BATTERY_NORMAL rated Voltage\n");
        szz= String(szz + "/" + "NORM_V").c_str();
        break;
           
    }

    switch (uiSBHN) {
      case BATTERY_OVER_TEMP:
        //printf("BATTERY Over temperature\n");
        szz= String(szz + "/" + "HIGH_T").c_str();
        break;
      case BATTERY_LOW_TEMP:
        //printf("ATTERY Low temperature\n");
        szz= String(szz + "/" + "LOW_T").c_str();
        break;   
      default:
        //printf("BATTERY_NORMAL temperature\n");
        szz= String(szz + "/" + "NORM_T").c_str();
        break;
           
    }

    szz = String(szz + "/" + ui).c_str();
    return szz;

    //printf("----\n");
}

String vAnalizaCharging(unsigned int ui,char* sz){
    unsigned int uiLBLN = ui & 0x000F;
    unsigned int uiLBHN = ui & 0x00F0;
    unsigned int uiSBLN = ui & 0x0F00;
    unsigned int uiSBHN = ui & 0xF000;
    String szz;
    
    //printf("%s:\n",sz);
    switch (uiLBLN) {
      case CHARGING_RUNNING:
        //printf("CHARGING_RUNNING\n");
        szz= String("RUN").c_str();
        break;
      case CHARGING_FAULT:
        //printf("CHARGING_FAULT\n");
        szz= String("FAIL").c_str();
        break;   
      case CHARGING_FLOAT:
        //printf("CHARGING_FLOAT\n");
        szz= String("FLOAT").c_str();
        break;
      case CHARGING_BOOST:
        //printf("CHARGING_BOOST\n");
        szz= String("BOOST").c_str();
        break;
      case CHARGING_EQUALIZATION:
        //printf("CHARGING_EQUALIZATION\n");
        szz= String("EQ").c_str();
        break;
      default:
        //printf("CHARGING_NORMAL\n");
        szz= String("NORM").c_str();
        break;
           
    }
    
    switch (uiLBHN) {
      case PV_INPUT_SHORTCIRCUIT:
        //printf("PV_INPUT_SHORTCIRCUIT\n");
        szz= String(szz + "/" + "IN_SHORT").c_str();
        break;
      case PV_DISEQUILIBRIUM:
        //printf("PV_DISEQUILIBRIUM\n");
        szz= String(szz + "/" + "NO_EQ").c_str();
        break;   
      default:
        //printf("PV_NORMAL\n");
        szz= String(szz + "/" + "PV_NORMAL").c_str();
        break;
           
    }

     switch (uiSBLN) {
      case MPPT_LOAD_MOSFET_SHORTCIRCUIT:
        //printf("MPPT_LOAD_MOSFET_SHORTCIRCUIT\n");
        szz= String(szz + "/" + "LOADM_SHORT").c_str();
        break;
      case MPPT_LOAD_SHORTCIRCUIT:
        //printf("MPPT_LOAD_SHORTCIRCUIT\n");
        szz= String(szz + "/" +  "LOAD_SHORT").c_str();
        break;   
      case MPPT_LOAD_OVERCURRENT:
        //printf("MPPT_LOAD_OVERCURRENT\n");
        szz= String(szz + "/" + "LOAD_HIGHA").c_str();
        break;
      case MPPT_INPUT_OVERCURRENT:
        //printf("MPPT_INPUT_OVERCURRENT\n");
        szz= String(szz + "/" + "INPUT_HIGHA").c_str();
        break;
      case MPPT_REVERSE_MOSFET_SHORTCIRCUIT:
        //printf("MPPT_REVERSE_MOSFET_SHORTCIRCUIT\n");
        szz= String(szz + "/" + "RWSHORT").c_str();
        break;
      default:
        //printf("MPPT_NORMAL\n");
        szz= String(szz + "/" + "NORM").c_str();
        break;
           
    }

    switch (uiSBHN) {
      case MPPT_REVERSE_MOSFET_OPENCIRCUIT:
        //printf("MPPT_REVERSE_MOSFET_OPENCIRCUIT\n");
        szz= String(szz + "/" + "OCIR").c_str();
        break;
      case MPPT_CHARGING_MOSFET_SHORTCIRCUIT:
        //printf("MPPT_CHARGING_MOSFET_SHORTCIRCUIT\n");
        szz= String(szz + "/" + "MOS_SHORT").c_str();
        break;   
      case MPPT_NO_INPUT_POWER:
        //printf("MPPT_NO_INPUT_POWER\n");
        szz= String(szz + "/" + "NO_INPUT").c_str();
        break;
      case MPPT_HIGH_INPUT_VOLTAGE:
        //printf("MPPT_HIGH_INPUT_VOLTAGE\n");
        szz= String(szz + "/" + "INPUT_HIGHV").c_str();
        break;
      case MPPT_INPUT_VOLTAGE_ERROR:
        //printf("MPPT_INPUT_VOLTAGE_ERROR\n");
        szz= String(szz + "/" + "INPUT_FAILV").c_str();
        break;
      default:
        //printf("MPPT__NORMAL\n");
        szz= String(szz + "/" + "NORMAL").c_str();
        break;
           
    }
    szz = String(szz + "/" + ui).c_str();
    return szz;

    //printf("----\n");
}


void vSupervisingButtons() {
  static boolean bIO0wasPressed = false;
  boolean bIO0currentState = bPressedButton(BT_IO0);
  static boolean bI34wasPressed = false;
  boolean bI34currentState = bPressedButton(BT_I34);
  static boolean bI35wasPressed = false;
  boolean bI35currentState = bPressedButton(BT_I35);

  if (bIO0wasPressed != bIO0currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_BT_IO0).c_str());
    if (bIO0currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_BT_IO0).c_str(), "IO0 pressed");
      Serial.println("Button IO0 pressed");
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_BT_IO0).c_str(), "IO0 released");
      Serial.println("Button IO0 released");
    }
    bIO0wasPressed = bIO0currentState;
  }
  if (bI34wasPressed != bI34currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_BT_I34).c_str());
    if (bI34currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_BT_I34).c_str(), "I34 pressed");
      Serial.println("Button I34 pressed");
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_BT_I34).c_str(), "I34 released");
      Serial.println("Button I34 released");
    }
    bI34wasPressed = bI34currentState;
  }
  if (bI35wasPressed != bI35currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_BT_I35).c_str());
    if (bI35currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_BT_I35).c_str(), "I35 pressed");
      Serial.println("Button I35 pressed");
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_BT_I35).c_str(), "I35 released");
      Serial.println("Button I35 released");
    }
    bI35wasPressed = bI35currentState;
  }


   
  //json_IoT["WIFI Status"] = WiFi.status();
  //json_IoT["WIFI IP"] = String(WiFi.localIP());
  //json_IoT["MAC Connected"] = sMac;
  json_IoT["bIO0"] = (bIO0currentState) ? "1" : "0";
  json_IoT["bI34"] = (bI34currentState) ? "1" : "0";
  json_IoT["bI35"] = (bI35currentState) ? "1" : "0";
  json_IoT["LDR"] = analogRead(LDR);
  json_IoT["T"] = ((float)nTx100_bme()/100.00) ;
  json_IoT["HR"] = ((float)nRHx100_bme()/100.00);
  json_IoT["P"] = ((float)nPx100_bme()/100.00);
  json_IoT["altura"] = ((float)nAx100_bme(SEALEVELPRESSURE_HPA)/100.00);
 
  json_IoT["LOAD_VOLTAGE"] = vGetMpptValues(0);
  json_IoT["LOAD_CURRENT"] = vGetMpptValues(1);
  json_IoT["LOAD_POWER"] = vGetMpptValues(2);
  json_IoT["PV_VOLTAGE"] = vGetMpptValues(3);
  json_IoT["PV_CURRENT"] = vGetMpptValues(4);
  json_IoT["PV_POWER"] = vGetMpptValues(5);
  json_IoT["BATTERY_VOLTAGE"] = vGetMpptValues(6);
  json_IoT["BATTERY_CURRENT"] = vGetMpptValues(7);
  json_IoT["BATTERY_SOC"] = vGetMpptValues(8);
  json_IoT["BATTERY_CAPACITY"] = vGetMpptValues(9);
  json_IoT["DATE"] = vGetMpptText(0);
  json_IoT["BATTERY_STATUS"] = vGetMpptText(1);
  //json_IoT["CHARGING_STATUS"] = vGetMpptText(2);
  
  
}


void vSetupModBus(int nBauds) {
  modbusData.begin(nBauds);          // U4_RXD:GPIO16, U4_TXD:GPIO17 (UART2)
  pinMode(SSerialTxControl, OUTPUT); // hardwired to GPIO_33
  vModeTxRxRS485(RS485Receive);
}

float vGetMpptValues(int n_enR) {
  unsigned char ucLastRx[MAX_STR_LEN];



  enum enReadings {read_LOAD_VOLTAGE, read_LOAD_CURRENT, read_LOAD_POWER, read_PV_VOLTAGE, read_PV_CURRENT, read_PV_POWER, read_BATTERY_VOLTAGE, read_BATTERY_CURRENT, read_BATTERY_SOC,read_BATTERY_CAPACITY};
  enReadings enR = (enReadings)n_enR;


  if (enR == read_LOAD_VOLTAGE) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_LOAD_VOLTAGE);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fLOADVOLTAGE= float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_LOAD_VOLTAGE:");
    //debug: Serial.print(fLOADVOLTAGE); Serial.println("V");
    String szLOADVOLTAGE = String(fLOADVOLTAGE);
    client.publish( String("/" + String(sMac) + TOPIC_LOAD_VOLTAGE).c_str(), szLOADVOLTAGE.c_str());
    return fLOADVOLTAGE;
  }
  if (enR == read_LOAD_CURRENT) {
    int nQ=0;
    int nQi=0;    
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_LOAD_CURRENT);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fLOADCURRENT= float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_LOAD_CURRENT:");
    //debug: Serial.print(fLOADCURRENT); Serial.println("A");
    String szLOADCURRENT = String(fLOADCURRENT);
    client.publish( String("/" + String(sMac) + TOPIC_LOAD_CURRENT).c_str(), szLOADCURRENT.c_str());
    return fLOADCURRENT;
    
  }
  if (enR == read_LOAD_POWER) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_LOAD_POWER);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<9)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fLOADPOWER = float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_LOAD_POWER:");
    //debug: Serial.print(fLOADPOWER); Serial.println("W");
    String szLOADPOWER = String(fLOADPOWER);
    client.publish( String("/" + String(sMac) + TOPIC_LOAD_POWER).c_str(), szLOADPOWER.c_str());
    return fLOADPOWER;
  }
  if (enR == read_PV_VOLTAGE) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_PV_VOLTAGE);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fPVVOLTAGE= float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_PV_VOLTAGE:");
    //debug: Serial.print(fPVVOLTAGE); Serial.println("V");
    String szPVVOLTAGE = String(fPVVOLTAGE);
    client.publish( String("/" + String(sMac) + TOPIC_PV_VOLTAGE).c_str(), szPVVOLTAGE.c_str());
    return fPVVOLTAGE;
  }
  if (enR == read_PV_CURRENT) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_PV_CURRENT);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fPVCURRENT= float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_PV_CURRENT:");
    //debug: Serial.print(fPVCURRENT); Serial.println("A");
    String szPVCURRENT = String(fPVCURRENT);
    client.publish( String("/" + String(sMac) + TOPIC_PV_CURRENT).c_str(), szPVCURRENT.c_str());
    return fPVCURRENT;
  }
  if (enR == read_PV_POWER) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_PV_POWER);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<9)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fPVPOWER = float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_PV_POWER:");
    //debug: Serial.print(fPVPOWER); Serial.println("W");
    String szPVPOWER = String(fPVPOWER);
    client.publish( String("/" + String(sMac) + TOPIC_PV_POWER).c_str(), szPVPOWER.c_str());
    return fPVPOWER;
  }
  if (enR == read_BATTERY_VOLTAGE) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_BATTERY_VOLTAGE);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fBATTERYVOLTAGE= float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_BATTERY_VOLTAGE:");
    //debug: Serial.print(fBATTERYVOLTAGE); Serial.println("V");
    String szBATTERYVOLTAGE = String(fBATTERYVOLTAGE);
    client.publish( String("/" + String(sMac) + TOPIC_BATTERY_VOLTAGE).c_str(), szBATTERYVOLTAGE.c_str());
    return fBATTERYVOLTAGE;
  }
  if (enR == read_BATTERY_CURRENT) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_BATTERY_CURRENT);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<9)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fBATTERYCURRENT = float(number)/100.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_BATTERY_CURRENT:");
    //debug: Serial.print(fBATTERYCURRENT); Serial.println("A");
    String szBATTERYCURRENT = String(fBATTERYCURRENT);
    client.publish( String("/" + String(sMac) + TOPIC_BATTERY_CURRENT).c_str(), szBATTERYCURRENT.c_str());
    return fBATTERYCURRENT;
  }
  if (enR == read_BATTERY_SOC) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_BATTERY_SOC);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fBATTERYSOC= float(number)/1.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_BATTERY_SOC:");
    //debug: Serial.print(fBATTERYSOC); Serial.println("%");
    String szBATTERYSOC = String(fBATTERYSOC);
    client.publish( String("/" + String(sMac) + TOPIC_BATTERY_SOC).c_str(), szBATTERYSOC.c_str());
    return fBATTERYSOC;
  }

    if (enR == read_BATTERY_CAPACITY) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_BATTERY_CAPACITY);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Sensor "); Serial.print(enR); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    float fBATTERYCAPACITY= float(number)/1.00;
    //debug: Serial.print("Sensor "); Serial.print(enR); Serial.print(" read_BATTERY_CAPACITY:");
    //debug: Serial.print(fBATTERYCAPACITY); Serial.println("Ah");
    String szBATTERYCAPACITY = String(fBATTERYCAPACITY);
    client.publish( String("/" + String(sMac) + TOPIC_BATTERY_CAPACITY).c_str(), szBATTERYCAPACITY.c_str());
    return fBATTERYCAPACITY;
  }
  
}



String vGetMpptText(int n_enRT) {
  unsigned char ucLastRx[MAX_STR_LEN];

  enum enTextReadings {read_DATE,read_BATTERY_STATUS,read_CHARGING_STATUS};
  enTextReadings enRT = (enTextReadings)n_enRT;
 
 if (enRT == read_DATE) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_DATE);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<11)&&(nQi<5));
    if (nQi>5){
      Serial.print("Text "); Serial.print(enRT); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    String sz = "20";
    if(ucLastRx[7] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[7],DEC);
    if(ucLastRx[8] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[8],DEC);        
    if(ucLastRx[5] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[5],DEC);
    if(ucLastRx[6] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[6],DEC);        
    if(ucLastRx[3] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[3],DEC);
    if(ucLastRx[4] < 10){
      sz+="0";
    }
    sz+= String(ucLastRx[4],DEC);          
    //debug: Serial.print("read_DATE "); Serial.print(enRT); Serial.print(":");
    //debug: Serial.println(sz);
    client.publish( String("/" + String(sMac) + TOPIC_DATE).c_str(), sz.c_str());
    return sz;
  }

  if (enRT == read_BATTERY_STATUS) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_BATTERY_STATUS);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Text "); Serial.print(enRT); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);
    String szBatteryStatus = vAnalizaBattery(number,"read_BATTERY_STATUS");
    //debug: Serial.print("Sensor "); Serial.print(enRT); Serial.print(" read_BATTERY_STATUS:");
    //debug: Serial.print(szBatteryStatus); Serial.println("%");
    client.publish( String("/" + String(sMac) + TOPIC_BATTERY_STATUS).c_str(), szBatteryStatus.c_str());
    return szBatteryStatus;
  }
  
  if (enRT == read_CHARGING_STATUS) {
    int nQ=0;
    int nQi=0;
    do {
      nQi++;
      //debug: Serial.print(nQi); Serial.println("reading trials");
      vPeticioModBus(MB_CHARGING_STATUS);
      nQ = nReceivingData(ucLastRx, TIMEOUT_ms);
    } while ((nQ<7)&&(nQi<5));
    if (nQi>5){
      Serial.print("Text "); Serial.print(enRT); Serial.print(" too many trials to recieve modbus");
    }
    int number = nShowReceivedData(ucLastRx,nQ);    
    String szChargingStatus = vAnalizaCharging(number,"read_CHARGING_STATUS");
    //debug: Serial.print("Sensor "); Serial.print(enRT); Serial.print(" read_CHARGING_STATUS:");
    //debug: Serial.print(szChargingStatus); Serial.println("%");
    client.publish( String("/" + String(sMac) + TOPIC_CHARGING_STATUS).c_str(), szChargingStatus.c_str());
    return szChargingStatus;
  }
    

}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  float fTc;
  float fRH;
  float fP;
  float fAlt;

  String szTopic = String(topic), szPayload = "";
  Serial.print("Topic: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    szPayload += (char)payload[i];
  }
  Serial.println();
  //Serial.print("Topic: "); Serial.println(szTopic);
  if (szTopic == TOPIC_REQUEST_MAC) {
    if (bPressedButton(BT_IO0)) {
      //szGetMac().toCharArray(sMac, MAC_SIZE);
      Serial.print(TOPIC_MAC); Serial.print(" : "); Serial.println(sMac);
      client.publish(TOPIC_MAC, sMac);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_LED_W).c_str()) {
    if ((char)payload[0] == '1') {
      digitalWrite(LED_W, HIGH);
    } else {
      digitalWrite(LED_W, LOW);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_LED_R).c_str()) {
    if ((char)payload[0] == '1') {
      digitalWrite(LED_R, HIGH);
    } else {
      digitalWrite(LED_R, LOW);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_LED_Y).c_str()) {
    if ((char)payload[0] == '1') {
      digitalWrite(LED_Y, HIGH);
    } else {
      digitalWrite(LED_Y, LOW);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_LED_G).c_str()) {
    if ((char)payload[0] == '1') {
      digitalWrite(LED_G, HIGH);
    } else {
      digitalWrite(LED_G, LOW);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_LATCHING_RELAY).c_str()) {
    if ((char)payload[0] == '1') {
      vLatchingRelay(true);
    } else {
      vLatchingRelay(false);
    }
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_LATCHING_RELAY_STATE).c_str()) {
    String szRelay = digitalRead(STATE_LATCHING_RELAY) ? "SET" : "RESET";
    Serial.print("Relay "); Serial.println(szRelay);
    client.publish( String("/" + String(sMac) + TOPIC_LATCHING_RELAY_STATE).c_str(), szRelay.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_LDR).c_str()) {
    String szLdr = String(analogRead(LDR));
    Serial.print("LDR: "); Serial.println(szLdr);
    client.publish( String("/" + String(sMac) + TOPIC_LDR).c_str(), szLdr.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_T).c_str()) {
    fTc = ((float)nTx100_bme()) / 100;
    String szTemp = String(fTc);
    Serial.print("T: "); Serial.println(szTemp);
    client.publish( String("/" + String(sMac) + TOPIC_T).c_str(), szTemp.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_RH).c_str()) {
    fRH = ((float) nRHx100_bme())/100;
    String szRH = String(fRH);
    Serial.print("RH: "); Serial.print(szRH); Serial.println(" %");
    client.publish( String("/" + String(sMac) + TOPIC_RH).c_str(), szRH.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_P).c_str()) {
    fP = ((float) nPx100_bme())/100;
    String szP = String(fP);
    Serial.print("P: "); Serial.print(szP); Serial.println(" hPa");
    client.publish( String("/" + String(sMac) + TOPIC_P).c_str(), szP.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_ALT).c_str()) {
    fAlt = ((float) nAx100_bme(SEALEVELPRESSURE_HPA))/100;
    String szAlt = String(fAlt);
    Serial.print("A: "); Serial.print(szAlt); Serial.println(" m");
    client.publish( String("/" + String(sMac) + TOPIC_ALT).c_str(), szAlt.c_str());
  } 
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_LOAD_VOLTAGE).c_str()) {
    Serial.print("Reading LOAD_VOLTAGE: ");
    vGetMpptValues(0);
  }

    if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_LOAD_CURRENT).c_str()) {
    Serial.print("Reading LOAD_CURRENT: ");
    vGetMpptValues(1);
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_LOAD_POWER).c_str()) {
    Serial.print("Reading LOAD_POWER: ");
    vGetMpptValues(2);    
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_PV_VOLTAGE).c_str()) {
    Serial.print("Reading PV_VOLTAGE: ");
    vGetMpptValues(3);    
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_PV_CURRENT).c_str()) {
    Serial.print("Reading PV_CURRENT: ");
    vGetMpptValues(4);    
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_PV_POWER).c_str()) {
    Serial.print("Reading PV_POWER: ");
    vGetMpptValues(5);  
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_BATTERY_VOLTAGE).c_str()) {
    Serial.print("Reading BATTERY_VOLTAGE: ");
    vGetMpptValues(6);  
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_BATTERY_CURRENT).c_str()) {
    Serial.print("Reading BATTERY_CURRENT: ");
    vGetMpptValues(7); 
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_BATTERY_SOC).c_str()) {
    Serial.print("Reading BATTERY_SOC: ");
    vGetMpptValues(8); 
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_DATE).c_str()) {
    Serial.print("Reading DATE: ");
    vGetMpptValues(9);
  }
  
  if (szTopic == String("/" + String(sMac) + TOPIC_BIG_TEXT).c_str()) {
    Serial.print("Big text: "); Serial.println(szPayload);
    vScreen24pixelText(0, 30, szPayload);
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_MEDIUM_TEXT).c_str()) {
    Serial.print("Medium text: "); Serial.println(szPayload);
    vScreen16pixelText(0, 30, szPayload);
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_SMALL_TEXT).c_str()) {
    Serial.print("Small text: "); Serial.println(szPayload);
    vScreen10pixelText(0, 30, szPayload);
  }
    
  if (szTopic == String("/" + String(sMac) + TOPIC_JSON_INPUT_REQ).c_str()) {
    String szJson = JSON.stringify(json_IoT);
    Serial.print("JSON: "); Serial.println(szJson);
    client.publish( String("/" + String(sMac) + TOPIC_JSON_INPUT).c_str(), szJson.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_JSON_LEDS  ).c_str()) {
    JSONVar myObject = JSON.parse(szPayload);
    if (JSON.typeof(myObject) == "undefined") {
      Serial.println("Parsing input failed!");
      return;
    }
    if (myObject.hasOwnProperty("ledW")) {
      //Serial.print("myObject[\"ledW\"] = ");
      //Serial.println((bool) myObject["ledW"]);
      digitalWrite(LED_W, (bool)myObject["ledW"]);
    }    
    if (myObject.hasOwnProperty("ledR")) {
      //Serial.print("myObject[\"ledR\"] = ");
      //Serial.println((int) myObject["ledR"]);
      digitalWrite(LED_R, (int)myObject["ledR"]);
    }
    if (myObject.hasOwnProperty("ledY")) {
      //Serial.print("myObject[\"ledY\"] = ");
      //Serial.println((int) myObject["ledY"]);
      digitalWrite(LED_Y, (int)myObject["ledY"]);
    }
    if (myObject.hasOwnProperty("ledG")) {
      //Serial.print("myObject[\"ledG\"] = ");
      //Serial.println((int) myObject["ledG"]);
      digitalWrite(LED_G, (int)myObject["ledG"]);
    }
  }
}

void mqttconnect() {
  /* Loop until reconnected */
  int n = 0;
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "IoT-02_" + String(sMac); // <-------   Unique name in every device
    /* connect now */
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      //if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic */
      client.subscribe(TOPIC_REQUEST_MAC); // <-------   Subscription to MQTT(S) topic
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_W).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_R).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_Y).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_G).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LATCHING_RELAY).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_LATCHING_RELAY_STATE).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_LDR).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_T).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_SMALL_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_MEDIUM_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_BIG_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_JSON_INPUT_REQ).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_JSON_LEDS).c_str());
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
      n++;
      Serial.print("trial");
      Serial.println(n);
    }
    if (n>5){
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      n=0;
    }
  }
}

void vSetupMqtt() {
  /* set SSL/TLS certificate */
  //espClient.setCACert(ca_cert);
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, mqtt_port);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);
}


void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.println(__FILE__);
  vSetupIO();
  vSetupWifi();
  szGetMac().toCharArray(sMac, MAC_SIZE);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(sMac);
  vSetupMqtt();
  vSetupScreen();
  vSetupBME280();
  vSetupModBus(BAUD_MPPT);
}




void loop() {
unsigned long currentMillis = millis();
  static unsigned long previousMillis = currentMillis;
  
  if((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=RECONNECTING_INTERVAL)){
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;    
  }else{
    /* if client was disconnected then try to reconnect again */
    if (!client.connected()) {
      mqttconnect();
    }else{
      /* this function will listen for incomming
        subscribed topic-process-invoke receivedCallback */
      client.loop();
    }
  }
  vSupervisingButtons();

    
    //Serial.println("Hola");
/*
  int i;

  for (i = 0; i < 10; i++) {
    Serial.print("Sensor "); Serial.print(i); Serial.println(":");
    vGetMpptValues(i);
    delay(1000);
  }

    for (i = 0; i < 4; i++) {
    Serial.print("Text "); Serial.print(i); Serial.println(":");
      vGetMpptText(i);
    delay(1000);
  }

*/

}
