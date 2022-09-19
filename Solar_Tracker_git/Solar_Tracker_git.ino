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

#include <WiFi.h>
#include "IoT-02_wifiMng.h"
#include "IoT-02_mqttCredentials.h"
#include "IoT-02_mqttTopics.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <Arduino_JSON.h>
#include <NTPClient.h>
#include <ESP32Time.h>
#include <WiFiUdp.h>

// Define NTP Client to get time
WiFiUDP ntpUDP;
const long utcOffsetInSeconds = 7200;
NTPClient timeClient(ntpUDP,"pool.ntp.org", utcOffsetInSeconds);

// Variables to save date and time
String formattedDate;
String today;

/* create an instance of WiFiClientSecure */
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);
#define RECONNECTING_INTERVAL 30000

#define MAC_SIZE 15
char sMac[MAC_SIZE];

JSONVar json_IoT;

#include <ESP32Time.h>
#include <NTPClient.h>
ESP32Time rtc(0);  // offset in seconds GMT+2

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;



//LDRs
#define LDR_TOP_LEFT 35
#define LDR_TOP_RIGHT 34
#define LDR_BOTTOM_LEFT 32
#define LDR_BOTTOM_RIGHT 33

const int LIGHT_THRESHOLD = 25;

//Servos
#define SERVO_H 26
#define SERVO_V 25
Servo servo_horizontal;
Servo servo_vertical;

const int LOWER_LIMIT_POS_H=2;  //Límite superior de los servos Horizontal
const int UPPER_LIMIT_POS_H=178;   //Límite inferior de los servos Horizontal
const int LOWER_LIMIT_POS_V=2;  //Límite superior de los servos Vertical
const int UPPER_LIMIT_POS_V=88;   //Límite inferior de los servos Vertical
int pos_sh;
int pos_sv;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
int opt_pos_H = 0;
int opt_pos_V = 0;
String Date;
int ldr_tl_value;
int ldr_tr_value;
int ldr_bl_value;
int ldr_br_value;
int diffV;
int diffH;



void vJson() {
  
  
  json_IoT["Pos_Horizontal"] = ((float)opt_pos_H) ;
  Serial.print("Optimal Position Horizontal: ");
  Serial.println(((float)opt_pos_H));
  json_IoT["Pos_Vertical"] = ((float)opt_pos_V) ;
  Serial.print("Optimal Position Vertical: ");
  Serial.println(((float)opt_pos_V));
  json_IoT["LED_TL"] = ((float)ldr_tl_value) ;
  Serial.print("LED TOP LEFT: ");
  Serial.println(((float)ldr_tl_value));
  json_IoT["LED_TR"] = ((float)ldr_tr_value) ;
  Serial.print("LED TOP RIGHT: ");
  Serial.println(((float)ldr_tr_value));
  json_IoT["LED_BL"] = ((float)ldr_bl_value) ;
  Serial.print("LED BOTTOM LEFT: ");
  Serial.println(((float)ldr_bl_value));
  json_IoT["LED_BR"] = ((float)ldr_br_value) ;
  Serial.print("LED BOTTOM RIGHT: ");
  Serial.println(((float)ldr_br_value));
  json_IoT["shuntvoltage"] = shuntvoltage ;
  Serial.print("Optimal pos shuntvoltage: ");
  Serial.println(shuntvoltage);
  json_IoT["busvoltage"] = busvoltage ;
  Serial.print("Optimal pos busvoltage: ");
  Serial.println(busvoltage); 
  json_IoT["current_mA"] = current_mA ;
  Serial.print("Optimal pos current_mA: ");
  Serial.println(current_mA); 
  json_IoT["loadvoltage"] = loadvoltage ;
  Serial.print("Optimal pos loadvoltage: ");
  Serial.println(loadvoltage); 
  json_IoT["power_mW"] = power_mW ;
  Serial.print("Optimal pos power_mW: ");
  Serial.println(power_mW); 
  json_IoT["Date"] = rtc.getDateTime(true) ;
  Serial.print("Date: ");
  Serial.println(rtc.getDateTime(true)); 
  
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
  
}



void receivedCallback(char* topic, byte* payload, unsigned int length) {

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


  if (szTopic == String("/" + String(sMac) + TOPIC_JSON_INPUT_REQ).c_str()) {
    String szJson = JSON.stringify(json_IoT);
    Serial.print("JSON: "); Serial.println(szJson);
    client.publish( String("/" + String(sMac) + TOPIC_JSON_INPUT).c_str(), szJson.c_str());
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
      client.subscribe(String("/" + String(sMac) + TOPIC_JSON_INPUT_REQ).c_str());
  
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
  // put your setup code here, to run once:
  
  
  Serial.begin(115200);
  Serial.println(__FILE__);
  vSetupWifi();
  szGetMac().toCharArray(sMac, MAC_SIZE);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(sMac);
 // Initialize a NTPClient to get time
  timeClient.begin();
  delay ( 500 );
  timeClient.update();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
  //rtc.setTime(00, 59, 1, 17, 1, 2021);// sec , min, hour, day, month, year
  rtc.setTime(timeClient.getEpochTime());//epoch time
  today = rtc.getDate(true);
  Serial.println("Today: ");
  Serial.println(today);
  

  vSetupMqtt();
  

  servo_vertical.attach(SERVO_V);
  servo_horizontal.attach(SERVO_H);
  int originPosH = 175;
  int originPosV = (LOWER_LIMIT_POS_V+UPPER_LIMIT_POS_V)/2;

  //Serial.println("move home: ");
  servo_horizontal.write(originPosH);
  servo_vertical.write(originPosV);
  pos_sv = servo_vertical.read();
  pos_sh = servo_horizontal.read();

  Serial.print("move Home, vertical pos:");
  Serial.println(pos_sv);
  Serial.print("move Home, horizontal pos:");
  Serial.println(pos_sh);

  uint32_t currentFrequency;
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("Measuring voltage and current with INA219 ...");
  delay(1000);

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
    // Initialize a NTPClient to get time
    timeClient.begin();
    delay ( 500 );
    timeClient.update();
    rtc.setTime(timeClient.getEpochTime());//epoch time
    Serial.println("Today reconnected: ");
    Serial.println(rtc.getDate(true));    
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
  
  
  //Leemos los 4 LDRs
  ldr_tl_value = analogRead(LDR_TOP_LEFT);
  ldr_tr_value = analogRead(LDR_TOP_RIGHT);
  ldr_bl_value = analogRead(LDR_BOTTOM_LEFT);
  ldr_br_value = analogRead(LDR_BOTTOM_RIGHT);

  int average_top = (ldr_tl_value + ldr_tr_value) / 2; //Media de los 2 LDR de arriba
  int average_bottom = (ldr_bl_value + ldr_br_value) / 2; //Media de los 2 LDR de abajo
  int average_left = (ldr_tl_value + ldr_bl_value) / 2; //Media de los 2 LDR de la izquierda
  int average_right = (ldr_tr_value + ldr_br_value) / 2; //Media de los 2 LDR de la derecha


  int minLDR = min(min(ldr_tl_value,ldr_tr_value),min(ldr_bl_value,ldr_br_value));
  Serial.print("minLDR ");
  Serial.println(minLDR);

  if(minLDR<150) {
    //Movemos el solar tracker
    moveSolarTracker(average_top, average_bottom, average_left, average_right);
    
  } else if (minLDR>150 && rtc.getDate(true)==today){
    opt_pos_H = pos_sh;
    opt_pos_V = pos_sv;
    currentMeasure();
    vJson();
    Serial.print("Optimal Position Horizontal ");
    Serial.println(opt_pos_H);
       
    Serial.print("Optimal Position Vertical ");
    Serial.println(opt_pos_V);
   
    Serial.print("LED TOP LEFT: ");
    Serial.println(((float)ldr_tl_value));
   
    Serial.print("LED TOP RIGHT: ");
    Serial.println(((float)ldr_tr_value));
    
    Serial.print("LED BOTTOM LEFT: ");
    Serial.println(((float)ldr_bl_value));
    
    Serial.print("LED BOTTOM RIGHT: ");
    Serial.println(((float)ldr_br_value));
    Serial.println("Today: ");
    Serial.println(today);
  
  }
    else if (minLDR>150 && rtc.getDate(true)!=today){
      
    int originPosH = 170;
    int originPosV = (LOWER_LIMIT_POS_V+UPPER_LIMIT_POS_V)/2;

    Serial.println("move home: ");
    servo_horizontal.write(originPosH);
    servo_vertical.write(originPosV);
    pos_sv = servo_vertical.read();
    pos_sh = servo_horizontal.read();
    opt_pos_H = pos_sh;
    opt_pos_V = pos_sv;
    currentMeasure();
    vJson();
    today = rtc.getDate(true);
    Serial.println("Today change: ");
    Serial.println(today);
    
  
    }
  

  

  //Delay de 30 ms para que los servos no se muevan demasiado rápido
  delay(1000);
}

void moveSolarTracker(int average_top, int average_bottom, int average_left, int average_right) {
  //Movemos el solar tracker hacia arriba o hacia abajo


      
  if (((average_top - average_bottom) > LIGHT_THRESHOLD && pos_sv < UPPER_LIMIT_POS_V) ||((average_top - average_bottom) > LIGHT_THRESHOLD && pos_sv > LOWER_LIMIT_POS_V)) {
    pos_sv--;
    servo_vertical.write(pos_sv);


    
  }
  else if (((average_bottom - average_top ) > LIGHT_THRESHOLD && pos_sv < UPPER_LIMIT_POS_V) ||((average_top - average_bottom) > LIGHT_THRESHOLD && pos_sv > LOWER_LIMIT_POS_V)) {
    pos_sv++;
    servo_vertical.write(pos_sv);



  }

  //Movemos el solar tracker hacia la derecha o hacia la izquierda
  if (((average_left - average_right) > LIGHT_THRESHOLD && pos_sh < UPPER_LIMIT_POS_H) || ((average_left - average_right) > LIGHT_THRESHOLD && pos_sh > LOWER_LIMIT_POS_H)) {
    pos_sh++;
    servo_horizontal.write(pos_sh);
 

  }
  else if (((average_right - average_left ) > LIGHT_THRESHOLD && pos_sh < UPPER_LIMIT_POS_H) || ((average_left - average_right) > LIGHT_THRESHOLD && pos_sh > LOWER_LIMIT_POS_H)) {
    pos_sh--;
    servo_horizontal.write(pos_sh);
   
  }
    Serial.print("HORIZONTAL ");
    Serial.println(average_left-average_right);
    Serial.print("VERTICAL ");
    Serial.println(average_top-average_bottom);

    
  
  if(abs(average_right - average_left)<LIGHT_THRESHOLD && abs(average_top - average_bottom)<LIGHT_THRESHOLD ){
    opt_pos_H = pos_sh;
    opt_pos_V = pos_sv;
    currentMeasure();
    vJson();
    Serial.print("Optimal Position Horizontal ");
    Serial.println(opt_pos_H);
       
    Serial.print("Optimal Position Vertical ");
    Serial.println(opt_pos_V);
   
    Serial.print("LED TOP LEFT: ");
    Serial.println(((float)ldr_tl_value));
   
    Serial.print("LED TOP RIGHT: ");
    Serial.println(((float)ldr_tr_value));
    
    Serial.print("LED BOTTOM LEFT: ");
    Serial.println(((float)ldr_bl_value));
    
    Serial.print("LED BOTTOM RIGHT: ");
    Serial.println(((float)ldr_br_value));
    
    
  }



}



void currentMeasure() {

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  /*Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");*/

}
