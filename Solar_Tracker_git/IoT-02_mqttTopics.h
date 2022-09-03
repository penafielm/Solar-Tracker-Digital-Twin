#ifndef IOT_02_MQTT_TOPICS
#define IOT_02_MQTT_TOPICS

// Pub
#define TOPIC_MAC "/mac"
#define TOPIC_BT_IO0 "/btIO0"
#define TOPIC_BT_I34 "/btI34"
#define TOPIC_BT_I35 "/btI35"
#define TOPIC_LATCHING_RELAY_STATE "/latchingRelayState"
#define TOPIC_LDR "/ldr"
#define TOPIC_T "/temp"
#define TOPIC_JSON_INPUT "/jsonIn"
#define TOPIC_RH "/rh"
#define TOPIC_P "/pressure"
#define TOPIC_ALT "/altitude"

//Pub MODBUS

#define TOPIC_LOAD_VOLTAGE "/Load_Voltage"
#define TOPIC_LOAD_CURRENT "/Load_Current"
#define TOPIC_LOAD_POWER "/Load_Power"
#define TOPIC_PV_VOLTAGE "/PV_Voltage"
#define TOPIC_PV_CURRENT "/PV_Voltage"
#define TOPIC_PV_POWER "/PV_Voltage"
#define TOPIC_BATTERY_VOLTAGE "/Battery_Voltage" 
#define TOPIC_BATTERY_CURRENT "/Battery_Current" 
#define TOPIC_BATTERY_SOC"/Battery_SOC"
#define TOPIC_BATTERY_CAPACITY"/Battery_Capacity" 
#define TOPIC_DATE"/Date" 
#define TOPIC_BATTERY_STATUS"/Battery_Status" 
#define TOPIC_BATTERY_INT_STATUS"/Battery_Int_Status" 
#define TOPIC_CHARGING_STATUS"/Charging_Status" 
#define TOPIC_SOLAR_STATUS"/Solar_Status" 


// Sub
#define TOPIC_REQUEST_MAC "/macReq"
#define TOPIC_LED_W "/ledW"
#define TOPIC_LED_R "/ledR"
#define TOPIC_LED_Y "/ledY"
#define TOPIC_LED_G "/ledG"
#define TOPIC_LATCHING_RELAY "/latchingRelay"
#define TOPIC_REQUEST_LATCHING_RELAY_STATE "/latchingRelayStateReq"
#define TOPIC_REQUEST_LDR "/ldrReq"
#define TOPIC_REQUEST_T "/tempReq"
#define TOPIC_REQUEST_RH "/rhReq"
#define TOPIC_REQUEST_P "/pressureReq"
#define TOPIC_REQUEST_ALT "/altitudeReq"
#define TOPIC_SMALL_TEXT "/10pxTxt"
#define TOPIC_MEDIUM_TEXT "/16pxTxt"
#define TOPIC_BIG_TEXT "/24pxTxt"
#define TOPIC_JSON_INPUT_REQ "/jsonInReq"
#define TOPIC_JSON_LEDS "/jsonLeds"

//Sub MODBUS

#define TOPIC_REQUEST_LOAD_VOLTAGE "/Load_VoltageReq"
#define TOPIC_REQUEST_LOAD_CURRENT "/Load_CurrentReq"
#define TOPIC_REQUEST_LOAD_POWER "/Load_PowerReq"
#define TOPIC_REQUEST_PV_VOLTAGE "/PV_VoltageReq"
#define TOPIC_REQUEST_PV_CURRENT "/PV_VoltageReq"
#define TOPIC_REQUEST_PV_POWER "/PV_VoltageReq"
#define TOPIC_REQUEST_BATTERY_VOLTAGE "/Battery_VoltageReq" 
#define TOPIC_REQUEST_BATTERY_CURRENT "/Battery_CurrentReq" 
#define TOPIC_REQUEST_BATTERY_SOC"/Battery_SOCReq" 
#define TOPIC_REQUEST_DATE"/DateReq" 

#endif // IOT_02_MQTT_TOPICS
