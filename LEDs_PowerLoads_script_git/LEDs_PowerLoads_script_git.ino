
// https://github.com/penafielm/Solar-Tracker-Digital-Twin
// 
//
// Code written by Ignacio Peñafiel Miguel and mentorship by Jordi Binefa. 20220901
// https://github.com/jordibinefa/IoT-02/tree/master/codes
// things.cat
//


#include <Arduino.h>
#define RELAY_LED1_PIN 23
#define RELAY_LED2_PIN 22
#define RELAY_LED3_PIN 21

#define RECONNECTING_INTERVAL 30000


#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifiMng.h"
#include "mqttCredentials.h"
#include "mqttTopics.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <Arduino_JSON.h>

//create time zone
#include <random>
#include <ESP32Time.h>
#include <NTPClient.h>
ESP32Time rtc(0);  // offset in seconds GMT+2
String szTime1;
String szTime2;
String szTime3;
// Define NTP Client to get time
WiFiUDP ntpUDP;
const long utcOffsetInSeconds = 7200;
NTPClient timeClient(ntpUDP,"pool.ntp.org", utcOffsetInSeconds);

// Variables to save date and time
String formattedDate;


/* create an instance of WiFiClientSecure */
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);

#define MAC_SIZE 15
char sMac[MAC_SIZE];
char sMobus[16];

JSONVar json_IoT;

boolean bLedsState(int nWhichOne) {
  if (digitalRead(nWhichOne))
    return true;
  return false;
}

void vSupervisingLeds() {
  static boolean LED1wasPressed = false;
  boolean LED1currentState = bLedsState(RELAY_LED1_PIN);
  static boolean LED2wasPressed = false;
  boolean LED2currentState = bLedsState(RELAY_LED2_PIN);
  static boolean LED3wasPressed = false;
  boolean LED3currentState = bLedsState(RELAY_LED3_PIN);

  
  if (LED1wasPressed != LED1currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_LED1_STATE).c_str());
    if (LED1currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_LED1_STATE).c_str(), "LED1 On");
      Serial.println("Button LED1 On");
    
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_LED1_STATE).c_str(), "LED1 Off");
      Serial.println("Button LED1 Off");
    }
    LED1wasPressed = LED1currentState;
  }
  if (LED2wasPressed != LED2currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_LED2_STATE).c_str());
    if (LED2currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_LED2_STATE).c_str(), "LED2 On");
      Serial.println("Button LED2 On");
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_LED2_STATE).c_str(), "LED2 Off");
      Serial.println("Button LED2 Off");
    }
    LED2wasPressed = LED2currentState;
  }
  if (LED3wasPressed != LED3currentState) {
    delay(2);
    Serial.print("Publishing topic: "); Serial.println(String("/" + String(sMac) + TOPIC_LED3_STATE).c_str());
    if (LED3currentState) {
      client.publish(String("/" + String(sMac) + TOPIC_LED3_STATE).c_str(), "LED3 On");
      Serial.println("Button LED3 On");
    } else {
      client.publish(String("/" + String(sMac) + TOPIC_LED3_STATE).c_str(), "LED3 Off");
      Serial.println("Button LED3 Off"); 
    }
    LED3wasPressed = LED3currentState;
  }

  json_IoT["LED1State"] = (LED1currentState) ? "On" : "Off";
  //debugSerial.print("LED1: ");
  //debugSerial.println(LED1currentState);
  json_IoT["LED1Time"] = szTime1;
  //debugSerial.print("LED1Time: ");
  //debugSerial.println(szTime1);
  json_IoT["LED2State"] =(LED2currentState) ? "On" : "Off";
  //debugSerial.print("LED2: ");
  //debugSerial.println(LED2currentState);
  json_IoT["LED2Time"] = szTime2;
  //debugSerial.print("LED2Time: ");
  //debugSerial.println(szTime2);
  json_IoT["LED3State"] = (LED3currentState) ? "On" : "Off";
  //debugSerial.print("LED3: ");
  //debugSerial.println(LED3currentState);
  json_IoT["LED3Time"] = szTime3;
  //debugSerial.print("LED3Time: ");
  //debugSerial.println(szTime3);

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
  if (szTopic == TOPIC_REQUEST_MAC) {
    Serial.print(TOPIC_MAC); Serial.print(" : "); Serial.println(sMac);
    client.publish(TOPIC_MAC, sMac);
  }
    
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
    String clientId = "" + String(sMac); // <-------   Unique name in every device
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

void vSetupIO() {
  pinMode(RELAY_LED1_PIN, OUTPUT);
  pinMode(RELAY_LED2_PIN, OUTPUT);
  pinMode(RELAY_LED3_PIN, OUTPUT);
}

void vHourlyConsumption(int nTimeLED1, int nTimeLED2, int nTimeLED3) {
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS; // 1 second
  digitalWrite(RELAY_LED1_PIN,HIGH);
  delay(1000);
  digitalWrite(RELAY_LED2_PIN,HIGH);
  delay(1000);
  digitalWrite(RELAY_LED3_PIN,HIGH);
  int nNow=millis();
  int currentHour = rtc.getHour(true);
  //Serial.print("Current hour: ");
  //Serial.println(currentHour);
  
  szTime1 = rtc.getDateTime();
  szTime2 = szTime1;
  szTime3 = szTime1; 
  
  do{
    if(millis()-nNow >= nTimeLED1 && bLedsState(RELAY_LED1_PIN)){
      digitalWrite(RELAY_LED1_PIN,LOW);
      szTime1 = rtc.getDateTime();
    }

    if(millis()-nNow >= nTimeLED2 && bLedsState(RELAY_LED2_PIN) ){
      digitalWrite(RELAY_LED2_PIN,LOW);
      szTime2 = rtc.getDateTime();   
    }
    if(millis()-nNow >= nTimeLED3 && bLedsState(RELAY_LED3_PIN)){
      digitalWrite(RELAY_LED3_PIN,LOW); 
      szTime3 = rtc.getDateTime(); 
    }
    //Serial.print("Current hour: ");
    //Serial.println(rtc.getHour(true));
    vSupervisingLeds();
    vTaskDelay(xDelay);
  } while (currentHour == rtc.getHour(true));


  //Serial.print("hour pass: ");
  //Serial.println(rtc.getHour(true));
  
  digitalWrite(RELAY_LED1_PIN,LOW);
  szTime1 = rtc.getDateTime();
  delay(500);
  digitalWrite(RELAY_LED2_PIN,LOW);
  szTime2 = rtc.getDateTime();
  delay(500);      
  digitalWrite(RELAY_LED3_PIN,LOW);
  szTime3 = rtc.getDateTime();
  
  vSupervisingLeds();
  
}

void setup() {
  // put your setup code here, to run once:


  
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
  Serial.println("Hour: ");
  Serial.println(rtc.getHour(true));
  Serial.println("Month: ");
  Serial.println(rtc.getMonth()+1);
  
  vSetupMqtt();


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

  
//December/January/Febraury
  if (rtc.getMonth()==11 || rtc.getMonth()==0 || rtc.getMonth()==1 ){
    if(rtc.getHour(true)==0 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);
    }
    if(rtc.getHour(true)==1 || rtc.getHour(true)==2 || rtc.getHour(true)==3 || rtc.getHour(true)==4 || rtc.getHour(true)==5|| rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;//
        int nMinLED1 = 999;//
        int nMaxLED2 = 150;//
        int nMinLED2 = 0;//0
        int nMaxLED3 = 150;//
        int nMinLED3 = 0;//0
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    
    }
    if(rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 250;
        int nMinLED2 = 0;
        int nMaxLED3 = 750;
        int nMinLED3 = 200;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);  
    }
    if(rtc.getHour(true)==7 || rtc.getHour(true)==8) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 750;
        int nMinLED3 = 500;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==9 || rtc.getHour(true)==10 || rtc.getHour(true)==11|| rtc.getHour(true)==12) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==13 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==14 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==15|| rtc.getHour(true)==16|| rtc.getHour(true)==17 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==18|| rtc.getHour(true)==19 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 250;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==20|| rtc.getHour(true)==21 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 750;
        int nMinLED3 = 250;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==22|| rtc.getHour(true)==23 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 750;
        int nMinLED3 = 100;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
  
  }


//March/April/May
  if (rtc.getMonth()==2 || rtc.getMonth()==3 || rtc.getMonth()==4){
    if(rtc.getHour(true)==0 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);
    }
    if(rtc.getHour(true)==1 || rtc.getHour(true)==2 || rtc.getHour(true)==3 || rtc.getHour(true)==4 || rtc.getHour(true)==5|| rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 150;
        int nMinLED2 = 0;
        int nMaxLED3 = 150;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    
    }
    if(rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 250;
        int nMinLED2 = 0;
        int nMaxLED3 = 750;
        int nMinLED3 = 150;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);  
    }
    if(rtc.getHour(true)==7 || rtc.getHour(true)==8) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 600;
        int nMinLED3 = 100;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==9 || rtc.getHour(true)==10 || rtc.getHour(true)==11|| rtc.getHour(true)==12) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==13 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==14 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==15|| rtc.getHour(true)==16|| rtc.getHour(true)==17 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==18|| rtc.getHour(true)==19 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 250;
        int nMaxLED3 = 250;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==20|| rtc.getHour(true)==21 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 750;
        int nMinLED3 = 250;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==22|| rtc.getHour(true)==23 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 750;
        int nMinLED3 = 100;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
  }

//June/July/August
  if (rtc.getMonth()==5 || rtc.getMonth()==6 || rtc.getMonth()==7){
    if(rtc.getHour(true)==0 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 150;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);
    }
    if(rtc.getHour(true)==1 || rtc.getHour(true)==2 || rtc.getHour(true)==3 || rtc.getHour(true)==4 || rtc.getHour(true)==5|| rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 150;
        int nMinLED2 = 0;
        int nMaxLED3 = 150;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    
    }
    if(rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 250;
        int nMinLED2 = 0;
        int nMaxLED3 = 750;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);  
    }
    if(rtc.getHour(true)==7 || rtc.getHour(true)==8) {
        int nMaxLED1 = 999;//999
        int nMinLED1 = 999;//999
        int nMaxLED2 = 999;//999
        int nMinLED2 = 500;//500
        int nMaxLED3 = 350;//350
        int nMinLED3 = 0;//0
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==9 || rtc.getHour(true)==10 || rtc.getHour(true)==11|| rtc.getHour(true)==12) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==13 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==14 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==15|| rtc.getHour(true)==16|| rtc.getHour(true)==17 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==18|| rtc.getHour(true)==19 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 250;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==20|| rtc.getHour(true)==21 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 250;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==22|| rtc.getHour(true)==23 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 500;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
  }


//september/October/November
  if (rtc.getMonth()==8 || rtc.getMonth()==9 || rtc.getMonth()==10){
    if(rtc.getHour(true)==0 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);
    }
    if(rtc.getHour(true)==1 || rtc.getHour(true)==2 || rtc.getHour(true)==3 || rtc.getHour(true)==4 || rtc.getHour(true)==5|| rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 150;
        int nMinLED2 = 0;
        int nMaxLED3 = 150;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    
    }
    if(rtc.getHour(true)==6 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 250;
        int nMinLED2 = 0;
        int nMaxLED3 = 750;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);  
    }
    if(rtc.getHour(true)==7 || rtc.getHour(true)==8) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 750;
        int nMinLED3 = 500;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==9 || rtc.getHour(true)==10 || rtc.getHour(true)==11|| rtc.getHour(true)==12) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==13 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==14 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==15|| rtc.getHour(true)==16|| rtc.getHour(true)==17 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 100;
        int nMaxLED3 = 100;
        int nMinLED3 = 0;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }
    if(rtc.getHour(true)==18|| rtc.getHour(true)==19 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 500;
        int nMinLED2 = 250;
        int nMaxLED3 = 750;
        int nMinLED3 = 250;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==20|| rtc.getHour(true)==21 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 999;
        int nMinLED2 = 500;
        int nMaxLED3 = 750;
        int nMinLED3 = 250;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);        
    }
    if(rtc.getHour(true)==22|| rtc.getHour(true)==23 ) {
        int nMaxLED1 = 999;
        int nMinLED1 = 999;
        int nMaxLED2 = 750;
        int nMinLED2 = 100;
        int nMaxLED3 = 750;
        int nMinLED3 = 100;
        
        int nTimeLED1 = random(nMinLED1,nMaxLED1)*3600;
        //Serial.println("random LED1");
        //Serial.println(nTimeLED1);
        int nTimeLED2 = random(nMinLED2,nMaxLED2)*3600;
        //Serial.println("random LED2");
        //Serial.println(nTimeLED2);
        int nTimeLED3 = random(nMinLED3,nMaxLED3)*3600;
        //Serial.println("random LED3");
        //Serial.println(nTimeLED3);

       vHourlyConsumption (nTimeLED1,nTimeLED2,nTimeLED3);       
    }

  }

}
