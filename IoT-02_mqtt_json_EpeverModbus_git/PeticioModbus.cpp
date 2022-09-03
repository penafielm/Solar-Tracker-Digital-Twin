#include "IoT-02_pinout.h"
#include "Arduino.h"
#include "RecieveModbus.h"

#define MODBUS_DELAY 2 // 20 at 4800baus and 2 at 115200
#define MAX_STR_LEN 50

HardwareSerial modbusData(2);
unsigned char ucLastMbMsg[MAX_STR_LEN];
int nMbLastLen;
bool bMbAnswer;

// Compute the MODBUS RTU CRC
// Adapted to Arduino from http://www.ccontrolsys.com/w/How_to_Compute_the_Modbus_RTU_Message_CRC
unsigned int uiModRTU_CRC(byte *buf, int len)
{
  unsigned int crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      { // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else           // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}


void vModeTxRxRS485(boolean bMode)
{
  digitalWrite(SSerialTxControl, bMode);
}


void vModbusTx(unsigned char *uc, int nLen)
{
  int i;

  vModeTxRxRS485(RS485Transmit);
  vTaskDelay(1);
  for (i = 0; i < nLen; i++)
  {
  modbusData.write(uc[i]);
  //Serial.print("uc (");
  //Serial.print(i);
  //Serial.print("): ");
  //Serial.println(uc[i],HEX);
  }
  vTaskDelay(MODBUS_DELAY); // 20 at 4800baus and 2 at 115200

  vModeTxRxRS485(RS485Receive);
  bMbAnswer = false;
  // uiMbTimeOut = millis() + TIMEOUT;
}


int str2hex(char *s)
{
  int x = 0;
  for (;;)
  {
    char c = *s;
    if (c >= '0' && c <= '9')
    {
      x *= 16;
      x += c - '0';
    }
    else if (c >= 'A' && c <= 'F')
    {
      x *= 16;
      x += (c - 'A') + 10;
    }
    else
      break;
    s++;
  }
  return x;
}

unsigned char vPeticioModBus(String address) {
  unsigned char ucSt[MAX_STR_LEN];
  String addressSubStr;
  char charBuf[3];
  int i, nMida = address.length() / 2;


  //debug: Serial.println("Petició a ModBus:");
  //debug: Serial.println(address);
  //debug: Serial.print("Amb velocitat i delay: ");
  //debug: Serial.println(MODBUS_DELAY);

  for (i = 0; i < nMida; i++)
  {
    addressSubStr = address.substring(2 * i, 2 + 2 * i);
    addressSubStr.toCharArray(charBuf, 3);
    ucSt[i] = str2hex(charBuf); 
    //Serial.print("ucSt[");
    //Serial.print(i);
    //Serial.print("] = ");
    //Serial.println(ucSt[i]);
  }

  unsigned int uiCRC = uiModRTU_CRC(ucSt, nMida);

  ucSt[nMida] = (byte)(uiCRC & 0xFF);
  ucSt[nMida + 1] = (byte)(uiCRC >> 8);

  vModbusTx(ucSt, nMida + 2);

}
