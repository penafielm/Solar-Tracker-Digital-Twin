#include "Arduino.h"

//numeric
#define MB_LOAD_VOLTAGE "0104310C0001"
#define MB_LOAD_CURRENT "0104310D0001"
#define MB_LOAD_POWER "0104310E0002"
#define MB_PV_VOLTAGE "010431000001"
#define MB_PV_CURRENT "010431010001"
#define MB_PV_POWER "010431020002"
#define MB_BATTERY_VOLTAGE "0104331A0001"
#define MB_BATTERY_CURRENT "0104331B0002"
#define MB_BATTERY_SOC "0104311A0001"
#define MB_BATTERY_CAPACITY "010390010001"

//text
#define MB_DATE "010390130003"
#define MB_BATTERY_STATUS "010432000001"
#define MB_CHARGING_STATUS "010432010001"


/* Python lecturaMB
#define LOAD_VOLTAGE (0x01,0x04,0x31,0x0C,0x00,0x01)
#define LOAD_CURRENT (0x01,0x04,0x31,0x0D,0x00,0x01)
#define LOAD_POWER (0x01,0x04,0x31,0x0E,0x00,0x02)
#define PV_VOLTAGE (0x01,0x04,0x31,0x00,0x00,0x01)
#define PV_CURRENT (0x01,0x04,0x31,0x01,0x00,0x01)
#define PV_POWER (0x01,0x04,0x31,0x02,0x00,0x02)
#define BATTERY_VOLTAGE (0x01,0x04,0x33,0x1A,0x00,0x01)
#define BATTERY_CURRENT (0x01,0x04,0x33,0x1B,0x00,0x02)
#define BATTERY_SOC (0x01,0x04,0x31,0x1A,0x00,0x01)
#define DATE (0x01,0x03,0x90,0x13,0x00,0x03)

python 070300000001
*/




extern HardwareSerial modbusData;

unsigned char vPeticioModBus(String address);
void vModbusTx(unsigned char *uc, int nLen);
void vModeTxRxRS485(boolean bMode);
