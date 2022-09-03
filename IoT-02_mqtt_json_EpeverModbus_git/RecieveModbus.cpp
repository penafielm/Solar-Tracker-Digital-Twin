#include "Arduino.h"
#include "PeticioModbus.h"
//HardwareSerial modbusData(2);
#define MAX_STR_LEN 50


// receive data with redudance

int nReceivingData(unsigned char *ucLastRx, int nTimeOut_ms) {
  unsigned long ulTimeReceived, ulTimeFuncStarted = millis();
  boolean bReceiveUntilTimeout = false, bFinished = false;
  int i = 0;

  while (!bFinished) {
    if (!bReceiveUntilTimeout) {
      if (modbusData.available() > 0 )  {
        i = 0;
        ucLastRx[i++] = modbusData.read();
        bReceiveUntilTimeout = true;
        ulTimeReceived = millis();
      } else {
        if (millis() - ulTimeFuncStarted >= nTimeOut_ms) {
          bFinished = true;
        }
      }
    } else {
      if (modbusData.available() > 0)    {
        ucLastRx[i++] = modbusData.read();
        ulTimeReceived = millis();        
      } else {        
        if (millis() - ulTimeReceived >= nTimeOut_ms)      {
          bReceiveUntilTimeout = false;
          bFinished = true;
        }
      }
    }
    if (i >= MAX_STR_LEN - 1){
      bFinished = true;
    }
  }
  return i;

}


int nShowReceivedData(unsigned char *ucLastRx, int nLen) {
  int i;
  //Serial.print("Recieved data in hexadecimal lenght: ");
  //Serial.println(nLen);
  for (int i = 0; i < nLen ; i++) {
    //debug: Serial.print(ucLastRx[i], HEX);
    //debug: Serial.print(" ");
  }
  //debug: Serial.println();
  //debug: Serial.print("Number of bytes to read: ");
  //debug: Serial.println(ucLastRx[2], DEC);
  //debug: Serial.print("Bytes to be interpreted: ");
  long int li = 0;
  for (i = 0; i < ucLastRx[2] ; i++) {
    //Serial.print(ucLastRx[3 + i], HEX);
    //Serial.print(" ");
  }
  if (ucLastRx[2] == 2) {
    for (i = 0; i < ucLastRx[2] ; i++) {
      li <<= 8;
      // Serial.print(ucLastRx[3 + i], HEX);
      li |= ucLastRx[3 + i];
      // Serial.print(" ");
    }
    //debug: Serial.println();
    //debug: Serial.print("Number: ");
    //debug: Serial.println(li, DEC);
    return li;
  } else {
    if (ucLastRx[2] == 4) {
      li = ucLastRx[5];
      li <<= 8;
      li |= ucLastRx[6];
      li <<= 8;
      li |= ucLastRx[3];
      li <<= 8;
      li |= ucLastRx[4];
      //debug: Serial.println();
      //debug: Serial.print("Number: ");
      //debug: Serial.println(li, DEC);
      return li;
    } else {
      if (ucLastRx[2] == 6) {
        /*debug: Serial.println();
        Serial.print("Minute: ");
        Serial.println(ucLastRx[3],DEC);
        Serial.print("Second: ");
        Serial.println(ucLastRx[4],DEC);
        Serial.print("Day: ");
        Serial.println(ucLastRx[5],DEC);
        Serial.print("Hour: ");
        Serial.println(ucLastRx[6],DEC);
        Serial.print("Year: ");
        Serial.println(ucLastRx[7],DEC);
        Serial.print("Month: ");
        Serial.println(ucLastRx[8],DEC);*/
      }
    }
  }

}
