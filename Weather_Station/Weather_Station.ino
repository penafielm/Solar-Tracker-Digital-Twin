
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

#include "IoT-02_bme280.h"
Adafruit_BME280 bme;


/* create an instance of WiFiClientSecure */
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);
#define RECONNECTING_INTERVAL 30000

#define MAC_SIZE 15
char sMac[MAC_SIZE];

JSONVar json_IoT;

void vJson() {

 
  /*json_IoT["T"] = ((float)nTx100_bme()/100.00) ;
  Serial.print("T: ");
  Serial.println(((float)nTx100_bme()/100.00));
  json_IoT["HR"] = ((float)nRHx100_bme()/100.00);
  Serial.print("HR: ");
  Serial.println(((float)nRHx100_bme()/100.00));
  json_IoT["P"] = ((float)nPx100_bme()/100.00);
  Serial.print("P: ");
  Serial.println(((float)nPx100_bme()/100.00));
  json_IoT["altura"] = ((float)nAx100_bme(SEALEVELPRESSURE_HPA)/100.00);
  Serial.print("altura: ");
  Serial.println(((float)nAx100_bme(SEALEVELPRESSURE_HPA)/100.00));*/
  json_IoT["wind"] = (float) calcWind ()/100.00;
  Serial.print("wind: ");
  Serial.println(json_IoT["wind"]);
  
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
    
  if (szTopic == String("/" + String(sMac) + TOPIC_JSON_INPUT_REQ).c_str()) {
    String szJson = JSON.stringify(json_IoT);
    Serial.print("JSON: "); Serial.println(szJson);
    client.publish( String("/" + String(sMac) + TOPIC_JSON_INPUT).c_str(), szJson.c_str());
  }
  
}

void mqttconnect() {
  /* Loop until reconnected */
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

int wind = 0;

unsigned long firstMillis = 0;                                       //Timers for the wind speed calculation
unsigned long lastMillis = 0;
unsigned long lastIntTime = 0;
int counter = 0;                                                     //Counter to keep track of the number of wind speed revolutions 

void IRAM_ATTR isr () {                                             //Interrupt routine, run with each reed switch interrupt

  unsigned long intTime = millis();
  if(intTime - lastIntTime > 150) {                                 //Debounce the reed switch input
    if (counter == 0){
      firstMillis = millis();
    }
    counter++;                                                       //Count each revolution
    lastMillis = millis();  
  }
  lastIntTime = intTime;                                             //Capture the first and last revolution time
}


int calcWind () {                                               //Function to calculate the wind speed
  int ave = 5000;
  if(counter != 0){
    ave = (lastMillis - firstMillis)/counter;   
  }
  Serial.println(counter);
  Serial.print("Average Tick Time: ");
  Serial.println(ave);
 
  int wind = (((2*3.14159*1000)/ave)*0.01)*1000;              //radius magnet to center is 10mm
  
  return wind; 
}


#define WIND 12

void vSetupIO() {
  pinMode(WIND, INPUT_PULLUP);
  attachInterrupt(WIND, isr, FALLING);
 
}

void setup() {
  //Wire.begin(I2C_SDA, I2C_SCL);
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
  //vSetupBME280();
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
  
  vJson();
     

}
