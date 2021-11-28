/*****************************************************************************
  This is an Oregon Scientific THGR810 / UVN8000 sensor emulator (Temperature,
  Humidity and UV) based on Joseph Shuy work: http://shuhy.com/esi/osv3_dock_sensor.htm

  Designed and modified by Daniele Sgroi, OS Protocol 3.0, (C)2017 daniele.sgroi@gmail.com

  Using task scheduler from Anatoli Arkhipenko: https://github.com/arkhipenko/TaskScheduler

  Based on libraries written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution 

  information on these sensor was gleened from multiple places
  - the "Oregon Scientific RF Protocol Description" from May 2011 has data packet format 
    information http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf

  - sourceforge.net forums discussion of the CRC in the data packet

  - crc calculation code was taken from crc8.c published by Rajiv Chakravorty
    http://www.rajivchakravorty.com/source-code/uncertainty/multimedia-sim/html/crc8_8c-source.html

  Parts used include:
  - Arduino Pro Mini - ATMEGA 168P - 3.3V - 8 MHz (voltage reg and LED removed)
  - generic 3-pin 433,92 MHz transmitter
  - BH1750 i2c Luxmeter
  - VEML6070 i2c Ultra-Violet radiation sensor
  - HTU21DF i2c Temperature and Humidity sensor (3.3V no i2c pullup)

Channel allocations (total of 7 measurements possible):

  0 UV Index
  2 Temperature and Humidity %
  3 Lux (sent scaled as Temperature) and Battery (sent scaled as Humidity %)

TARGET: ATmega 168 / 8 MHz @ 3.0V = 2.81 mA idle / 6.42 mA tx peak

WARNING: with _DEBUG_ enabled, sketch uses up to 80% of AT168P Dynamic Memory. 
         This is marginal for stability and resources availability. 
         Comment _DEBUG_ for release version.
         Upgrade to 328P should be also possible.

Tested succesfully with WMR88 and WMR200 OS base stations.

Last updated on November 28, 2021
  
***************************************************************************/

#define _DEBUG_                 // comment to reduce footprint
#define SW_VERSION      1.00      // November 28, 2021
#define ADC_VBAT_K      0.0495F   // ADC scalar factor for voltage sensing

/***************************************************************************
 * Defines - HW pin
***************************************************************************/

#define LUX_POWER_PIN      2 // lux+uv power
#define UV_POWER_PIN       3 // Provision, not used uv powered by lux
#define TH1_POWER_PIN      4 // th power
#define TH2_POWER_PIN      5 // provision, not used
#define TX_POWER_PIN       6 // 433,92 MHz TX Power
#define TX_DATA_PIN        7 // 433,92 MHz TX Data
//                         8 
#define HW_CFG_PIN         9 // Provision, GND = Enables features
//                        10 
//                        11 
//                        12
//#define LED_BUILTIN     13 // no need to redefine LED_BUILTIN
//                        A0
#define ANALOG_PIN_1      A1 // used for rolling code at startup
//                        A2
#define BATTERY_SENSE     A3 // used to measure battery level, with R scaling
#define I2C_SDA           A4
#define I2C_SCL           A5
//                        A6
//                        A7

/************************************************************** *************
 * Defines - HTU21DF Temperature and Humidity sensor stuff
***************************************************************************/ 

#define HTU21DF_I2C_ADDRESS         0x40
#define HTU21DF_READTEMP            0xE3
#define HTU21DF_READHUM             0xE5
#define HTU21DF_WRITEREG            0xE6
#define HTU21DF_READREG             0xE7
#define HTU21DF_RESET               0xFE

/***************************************************************************
 * Defines - VEML6070 UV meter stuff
***************************************************************************/ 

#define VEML6070_I2C_ADDRESS        0x38    // I2C address = 0x38 L / 0x39 H
#define VEML6070_05_T                  0    // 90 ms
#define VEML6070_1_T                   1    // 180 ms
#define VEML6070_2_T                   2    // 360 ms
#define VEML6070_4_T                   3    // 720 ms

/***************************************************************************
 * Defines - BH1750 Luxmeter stuff
***************************************************************************/ 

#define BH1750_I2C_ADDRESS          0x23    // The I2C address is 0x23 (ADDR pin is low) or 0x5C (ADDR pin is high).
#define BH1750_POWER_DOWN           0x00    // Stop all operations
#define BH1750_POWER_ON             0x01    // Power on and wait for measurement command
#define BH1750_RESET                0x07    // Clears the illuminance data register, does not work in power down.
#define BH1750_CONT_H_RES_MODE      0x10
#define BH1750_CONT_H_RES_MODE2     0x11
#define BH1750_CONT_L_RES_MODE      0x13
#define BH1750_ONE_TIME_H_RES_MODE  0x20
#define BH1750_ONE_TIME_H_RES_MODE2 0x21
#define BH1750_ONE_TIME_L_RES_MODE  0x23
#define BH1750_MEAS_TIME_H          0x40
#define BH1750_MEAS_TIME_L          0x60

// The default correction factor to calculate the lux is 1.2. 
// Adjust it when you are able to calibrate the BH1750.
// According to the datasheet, the value can be 0.96 to 1.44.
// It is not know if this correction factor is the same for the whole range.
#define BH1750_FACTOR               1.2F

/***************************************************************************
 * Defines - OS V3.0 packets
***************************************************************************/ 

#define THGR810_PACKET_LEN           13
#define UVN_800_PACKET_LEN           12

// Sensor channels to tx on
#define UV_CHNL                       0 // Trick!
#define TEMP_HUM_CHNL                 2 // OS Sensor channel
#define LUX_BAT_CHNL                  3 // OS Sensor channel

// Sensor channel tx rate in sec. idx 0 is UV rate
const unsigned long int channelrate[] = {73000, 53000, 59000, 61000, 67000, 
                                         71000, 79000, 83000, 87000, 91000, 
                                         93000};

/***************************************************************************
 * Defines - Scheduler modes
***************************************************************************/ 

#define _TASK_SLEEP_ON_IDLE_RUN       // enable sleep for low power mode

/***************************************************************************
 * Includes 
***************************************************************************/ 

#include <Wire.h>
#include "TaskScheduler.h" // https://github.com/arkhipenko/TaskScheduler

/***************************************************************************
 * Functions Prototypes 
***************************************************************************/ 

// HTU21DF
int HTU21DF_begin(void);
float HTU21DF_readTemperature(void);
int HTU21DF_readHumidity(void);
void HTU21DF_readStatus(void); // updates global Battery status

// VEML6070
int VEML6070_begin(unsigned char);
int VEML6070_shutdown(void);
uint16_t VEML6070_readUV(void);
unsigned char calcUVI(uint16_t);

// BH1750 
int BH1750_begin(void);
uint16_t BH1750_read16(void);
int BH1750_write8(uint8_t);
inline uint8_t BH1750_mt_H(uint8_t);
inline uint8_t BH1750_mt_L(uint8_t);

// I2C sensors power Management
void i2cSensorOn(void);
void i2cSensorOff(void);

// CRC Stuff
void initCrc8(void);
void crc8(unsigned char *, unsigned char);

/***************************************************************************
 * Scheduler stuff
***************************************************************************/

// Scheduler Callback methods prototypes
void t1Callback(void); // temp, hum
void t2Callback(void); // Lux, bat 
void t3Callback(void); // UVI

//Tasks
Task t1(channelrate[TEMP_HUM_CHNL], TASK_FOREVER, &t1Callback);
Task t2(channelrate[LUX_BAT_CHNL], TASK_FOREVER, &t2Callback); 
Task t3(channelrate[UV_CHNL], TASK_FOREVER, &t3Callback); 

// Scheduler
Scheduler runner;

/***************************************************************************
 * Globals
***************************************************************************/

// Temperature and Humidity + Status
float Temperature = 0.0;    // sent as temperature 1
int Humidity = 0;           // sent as humidity 1
unsigned char Battery = 0;  // 1 = Battery Good, sent as Battery status for ch 1 and 3

// lux
float lux = 0.0; // from 0 to 100k lux, sent as temperature 3
float vbat = 0.0; // from 0 to 100%, sent as hum 3

// UV
unsigned int UVLevel = 0; 
unsigned int UVIndex = 0; // sent as uvi 1

// OSV3 packet and rollingcode
unsigned char packet[16];
unsigned int rollingCode;
 
// CRC table 
unsigned char crc8Table[256];     /* 8-bit table */
int madeTable;

/***************************************************************************
 * HTU21DF_begin
 * 
 * Initialize the Temperature and Humidity sensor
 * No more error checks after this.
 * 
***************************************************************************/

int HTU21DF_begin(void) {

  // reset
  Wire.beginTransmission(HTU21DF_I2C_ADDRESS);
  Wire.write(HTU21DF_RESET);
  Wire.endTransmission();
  delay(15);

  // init
  Wire.beginTransmission(HTU21DF_I2C_ADDRESS);
  Wire.write(HTU21DF_READREG);
  Wire.endTransmission();
  Wire.requestFrom(HTU21DF_I2C_ADDRESS, 1);
  return (Wire.read() != 0x2); // after reset should be 0x2
  
}

/***************************************************************************
 * HTU21DF_readTemperature
 * 
 * Read the Temperature
 * 
***************************************************************************/

float HTU21DF_readTemperature(void) {

  Wire.beginTransmission(HTU21DF_I2C_ADDRESS);
  Wire.write(HTU21DF_READTEMP);
  Wire.endTransmission();
  
  delay(50); // add delay between request and actual read!


  if (Wire.requestFrom(HTU21DF_I2C_ADDRESS, 3) == 3) {
      
    uint16_t t = Wire.read();
    t <<= 8;
    t |= Wire.read();

    uint8_t crc = Wire.read();

    float temp = t * 175.72F;
    temp /= 65536.0F;
    temp -= 46.85F;

    return temp;
    
  }
   
}

/***************************************************************************
 * HTU21DF_readHumidity
 * 
 * Read the Humidity
 *
***************************************************************************/

int HTU21DF_readHumidity(void) {

    Wire.beginTransmission(HTU21DF_I2C_ADDRESS);
    Wire.write(HTU21DF_READHUM);
    Wire.endTransmission();

    delay(50); // add delay between request and actual read!

    if (Wire.requestFrom(HTU21DF_I2C_ADDRESS, 3) == 3) {

      uint16_t h = Wire.read();
      h <<= 8;
      h |= Wire.read();

      uint8_t crc = Wire.read();

      float hum = h * 125.0F;
      hum /= 65536.0F;
      hum -= 6.0F;

      if (hum > 99.9) hum = 99.9;

      if (hum < 0.1) hum = 0.1;

      return hum;

  }

}

/***************************************************************************
 * HTU21DF_readStatus
 * 
 * Read the sensor status, including battery
 *
***************************************************************************/

void HTU21DF_readStatus(void) {


  Wire.beginTransmission(HTU21DF_I2C_ADDRESS);
    Wire.write(HTU21DF_READREG);
    Wire.endTransmission();

    delay(50); // add delay between request and actual read!

    if (Wire.requestFrom(HTU21DF_I2C_ADDRESS, 1) == 1) {
      //bit 6 = 1 means battery depleted
      Battery = !(Wire.read() & 64 ); // 1 = good
  }

}

/***************************************************************************
 * VEML6070_begin
 * 
 * Initialize the UV sensor
 * No more error checks after this.
 * 
***************************************************************************/

int VEML6070_begin(unsigned char itime) {

  Wire.beginTransmission(VEML6070_I2C_ADDRESS);
  Wire.write((itime << 2) | 0x02);
  
  return(Wire.endTransmission());
  
}

/***************************************************************************
 * VEML6070_shutdown
 * 
 * shutdown the UV sensor
 * No more error checks after this.
 * 
***************************************************************************/

int VEML6070_shutdown() {

  Wire.beginTransmission(VEML6070_I2C_ADDRESS);
  Wire.write(0x03); // bit 0 means shutdown
  
  return(Wire.endTransmission());
  
}

/***************************************************************************
 * VEML6070_readUV
 * 
 * Read the two bytes from the UV sensor
 * If an error occurs, then zero is returned.
***************************************************************************/

uint16_t VEML6070_readUV(void) {
  
  if (Wire.requestFrom(VEML6070_I2C_ADDRESS+1, 1) != 1) return -1;
  uint16_t uvi = Wire.read();
  uvi <<= 8;
  if (Wire.requestFrom(VEML6070_I2C_ADDRESS, 1) != 1) return -1;
  uvi |= Wire.read();

  return uvi;  
}

/***************************************************************************
* calcUVI
* 
* Compute UV Index from UV Level
* 
* From Vishay AN84310, the following conversion applies for Rset = 270 kOhm
* and Integration Time = 1T = 125 ms
* 
***************************************************************************/

unsigned char calcUVI(uint16_t uvl) {

       if (uvl > 2799) return(15); //       > 2799, UVI = 15
  else if (uvl > 2613) return(14); // 2614 to 2799, UVI = 14
  else if (uvl > 2427) return(13); // 2428 to 2613, UVI = 13
  else if (uvl > 2241) return(12); // 2242 to 2427, UVI = 12
  else if (uvl > 2054) return(11); // 2055 to 2241, UVI = 11
  else if (uvl > 1867) return(10); // 1868 to 2054, UVI = 10
  else if (uvl > 1681)  return(9); // 1682 to 1867, UVI =  9
  else if (uvl > 1494)  return(8); // 1495 to 1681, UVI =  8
  else if (uvl > 1307)  return(7); // 1308 to 1494, UVI =  7
  else if (uvl > 1120)  return(6); // 1121 to 1307, UVI =  6
  else if (uvl >  933)  return(5); //  934 to 1120, UVI =  5
  else if (uvl >  747)  return(4); //  748 to  933, UVI =  4
  else if (uvl >  560)  return(3); //  561 to  747, UVI =  3
  else if (uvl >  373)  return(2); //  374 to  560, UVI =  2
  else if (uvl >  186)  return(1); //  187 to  373, UVI =  1                                                               
  else                  return(0); //        < 187, UVI =  0
  
}

/***************************************************************************
 * BH1750_begin
 * 
 * A 'RESET' is only valid with power on.
 * A 'RESET' only resets the illuminace data register.
 * 
 * Three checks if the communication with the sensor is okay.
 * No more error checks after this.
 * 
***************************************************************************/

int BH1750_begin() {

  int error = 0;
  
  error = BH1750_write8(BH1750_POWER_ON);
  //if(error != 0)
  //  return(error);
    
  error += BH1750_write8(BH1750_RESET);
  //if(error != 0)
  //  return(error);

  error += BH1750_write8(BH1750_POWER_DOWN);
  //if(error != 0)
  //  return(error);

  return(error);  
  
}

/***************************************************************************
 * BH1750_read16
 * 
 * Read the two bytes from the BH1750
 * If an error occurs, then zero is returned.
 * 
***************************************************************************/

uint16_t BH1750_read16() {
  
  if (Wire.requestFrom(BH1750_I2C_ADDRESS, 2) == 2) {
    byte highbyte = Wire.read();              // highest byte first
    byte lowbyte  = Wire.read();
    return(word(highbyte, lowbyte));
  } 
  else return(0);
  
}

/***************************************************************************
 * BH1750_write8
 * 
 * Write the command byte to the BH1750
 * An error is returned, it is zero for no error.
 * 
***************************************************************************/

int BH1750_write8( uint8_t command) {
  
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(command);
  
  return(Wire.endTransmission());
  
}

/***************************************************************************
 * BH1750_mt_H
 * 
***************************************************************************/

inline uint8_t BH1750_mt_H( uint8_t mt) {
  
  return( BH1750_MEAS_TIME_H | (mt >> 5));  // highest 3 bits
  
}

/***************************************************************************
 * BH1750_mt_L
 * 
***************************************************************************/

inline uint8_t BH1750_mt_L( uint8_t mt) {
  
  return( BH1750_MEAS_TIME_L | (mt & 0x1F));   // lowest 5 bits
  
}

/***************************************************************************
 * initCrc8
 * 
 * Should be called before any other crc function
 * this CRC code is from crc8.c published by Rajiv Chakravorty
 * 
 * x^8 + x^2 + x + 1
 * 
***************************************************************************/

void initCrc8(void) {

  int i,j;
  unsigned char crc;
  
  if (!madeTable) {
    for (i=0; i<256; i++) {
      crc = i;
      for (j=0; j<8; j++) {
        crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
      }
      crc8Table[i] = crc & 0xFF;
    }
    
    madeTable = true;
  
  }

}

/***************************************************************************
 * crc8
 * 
 * For a byte array whose accumulated crc value is stored in *crc, computes
 * resultant crc obtained by appending m to the byte array
 * 
***************************************************************************/

void crc8(unsigned char *crc, unsigned char m) {
  
  if (!madeTable)
    initCrc8();

  *crc = crc8Table[(*crc) ^ m];
  *crc &= 0xFF;

}

/***************************************************************************
 * calcChecksum
***************************************************************************/

unsigned char calcChecksum(unsigned int lenght) {
  
  unsigned char checksum;

  // does not include all of byte at index 3 (skip the sync pattern 0xA)
  checksum = (packet[3] & 0xF);

  // all of 4 - 10
  for (int i=4; i<=lenght-3; i++)  {
    checksum += (packet[i] & 0xF) + ((packet[i] >> 4)  & 0xF);
  }

  // nibble swap
  return ((checksum & 0x0F) << 4) | ((checksum & 0xF0) >> 4);
  
}

/***************************************************************************
 * calcCrc
***************************************************************************/

unsigned char calcCrc(unsigned int lenght) {
  
  unsigned char crc = 0;

  // does not include all of byte at index 3 (skip the sync pattern 0xA)
  crc8(&crc, packet[3] & 0x0F);

  // includes bytes at indicies 4 through 10, does not include index 11   
  for (int i=4; i<=(lenght-3); i++) {
     crc8(&crc, packet[i]);
  }

  // nibble swap
  return ((crc & 0x0F) << 4) | ((crc & 0xF0) >> 4);;  

}

/***************************************************************************
 * buildAndSendTempHum
***************************************************************************/

void buildAndSendTempHum(float tempInC, int humidity, unsigned char txchannel) {
  
  // Nibbles are sent LSB first
  //      PACKET  0  1  2  3  4  5  6  7  8  9 10 11 12 
  //THGR810 [SW] ff ff ff af 82 42 RR 80 33 06 60 d4 2b
  //THGR810 [RF] 00 00 00 5f 14 24 88 10 cc 06 60 dc 49
  //THGN800 [RF] 00 00 00 5f 14 28 7f 8a c4 00 e2 8a cb
  
  // The preamble consists of twenty four '1' bits (6 nibbles) for v3.0 sensors
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFF;

  // A sync nibble (4-bits) which is '0101'
  packet[3] = 0xA0; 

  // --- payload ---
  // sending THGR810 packet
  // nibbles 0..3 Sensor ID This 16-bit value is unique to each sensor, 
  // or sometimes a group of sensors, F824 = THGR810
  packet[3] |= 0x0F; // results AF
  packet[4] = 0x82;  // 0x82
  packet[5] = 0x40;  // 0x40

  // nibble 4 TX_CHANNEL 1 thru 15
  packet[5] |= txchannel; // 4

  // nibbles 5..6 Rolling Code Value changes randomly at every reset
  packet[6] = rollingCode; // 5..6

  // nibble 7 Flags - battery status - 7..8
  if (Battery == 1) {
    packet[7] = 0x80;
  }
  else {
    packet[7] = 0x00;
  }

  // nibble 8..[n-5] Sensor-specific Data
  // nibbles 10..8 Temperature LSD is 0.1 degC

  // change a 23.5 C float value to 235 long value
  long temp = tempInC * 10;
  
  // if temperature is 23.5, then pull out the 5
  packet[7] |= (abs(temp) % 10) & 0x0F;

  // if temperature is 23.5, then pull out the 3
  packet[8] = ((abs(temp) / 10) % 10) << 4;

  // if temperature is 23.5, then pull out the 2
  packet[8] |= (abs(temp) / 100) & 0x0F;

  // nibble 11 Temperature Sign Non-zero for negative values
  packet[9] = (tempInC < 0.0) ? 0x80 : 0;

  // nibbles 15..12 is humidity
  packet[9] |= (humidity % 10) & 0x0F;
  packet[10] = ((humidity / 10) % 10) << 4;
  
  // nibbles [n-3]..[n-4] Checksum The 8-bit sum of nibbles 0..[n-5]
  packet[11] = calcChecksum(THGR810_PACKET_LEN);
  
  // CRC-8
  packet[12] = calcCrc(THGR810_PACKET_LEN);

  // send
  sendData(THGR810_PACKET_LEN);

}

/***************************************************************************
 * buildAndSendUV
***************************************************************************/

void buildAndSendUV(unsigned char uvidx) {
  
  // Nibbles are sent LSB first
  //      PACKET  0  1  2  3  4  5  6  7  8  9 10 11  
  //                               RR UU 
  //        MSG   -  -  -  0  1  2  3  4  5  6  7  8    
  //        NIB           01 23 45 67 89 
  //UVN800  [SW] 00 00 00 5b 1e 28 3d 0a 01 a0 52 e1   UVI = 5
  //UVN800  [RF] 00 00 00 5b 1e 28 96 0a 06 d0 62 3a   UVI = 5
          
  //sensor_id = (msg[0] << 8) | msg[1];
  //uvidx = ((message[4]&0x0f)*10)+(message[4]>>4);
  
  // The preamble consists of twenty four '1' bits (6 nibbles) for v3.0 sensors
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFF;

  // A sync nibble (4-bits) which is '0101'
  packet[3] = 0xA0; 

  // --- payload ---
  // nibbles 0..3 Sensor ID, D874 = UVN800
  packet[3] |= 0x0D; // results AD //sensor_id = (packet[3] << 8) | packet[4];
  packet[4] = 0x87;  
  packet[5] = 0x40; 

  // nibble 4 TX_CHANNEL 1 thru 15
  packet[5] |= 1; // UVN800 TX on Channel 1

  // nibbles 5..6 Rolling Code Value changes randomly at every reset
  packet[6] = rollingCode; // msg[3]

  // my sensor is plugged in, so power is always good
  packet[7] = uvidx; // msg[4]

  packet[8] = 0x08; // who know?

  packet[9] = 0x50; // who know?

  packet[10] = calcChecksum(UVN_800_PACKET_LEN);
  
  // CRC-8
  packet[11] = calcCrc(UVN_800_PACKET_LEN);

  // send
  sendData(UVN_800_PACKET_LEN);

}

/***************************************************************************
 * manchesterEncode
***************************************************************************/

void manchesterEncode (unsigned char encodeByte, bool lastByte) {
  
  unsigned char mask = 0x10;
  int loopCount;
  static unsigned long baseMicros = micros();

  // 488us timing would be 1024 data rate
  // the data rate actually appears to be 1020 or 490us
  // if the timing is off, then the base station won't receive packets reliably
  const unsigned int desiredDelay = 488; // was 490

  // due to processing time, the delay shouldn't be a full desired delay time
  const unsigned int shorten = 28; // 16, original 32
  
  // bits are transmitted in the order 4 thru 7, then 0 thru 3
  
  for (loopCount = 0; loopCount < 8; loopCount++) {  
    baseMicros += desiredDelay;
    unsigned long delayMicros = baseMicros - micros();
    
    if (delayMicros > 2*delayMicros) {
      // big delay indicates break between packet transmits, reset timing base
      baseMicros = micros();
    }
    else if (delayMicros > 0) {
      delayMicroseconds(delayMicros); 
    }
    
    if ((encodeByte & mask) == 0) {
      // a zero bit is represented by an off-to-on transition in the RF signal
      digitalWrite(TX_DATA_PIN, LOW);
      delayMicroseconds(desiredDelay - shorten); 
      digitalWrite(TX_DATA_PIN, HIGH);     
      // no long delay after last low to high transistion as no more data follows
      if (lastByte) delayMicroseconds(desiredDelay); 
    }
    else {
      digitalWrite(TX_DATA_PIN, HIGH);
      delayMicroseconds(desiredDelay - shorten); 
      digitalWrite(TX_DATA_PIN, LOW);
    }

    if (mask == 0x80) {
      mask = 0x01;
    }
    else {
      mask <<= 1;
    }

    baseMicros += desiredDelay;
  
  }
  
}  

/***************************************************************************
 * sendData
***************************************************************************/

void sendData(unsigned char lenght) {
  
  unsigned char i;
  
  digitalWrite(TX_DATA_PIN, LOW);    
  digitalWrite(TX_POWER_PIN, HIGH);  // power on transmitter
  delay(10);                         // wait for transmitter to get ready
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
      
  for (i=0; i < lenght; i++) {
    manchesterEncode(packet[i], (i+1 == lenght));
  }

  // power off transmitter
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
  digitalWrite(TX_DATA_PIN, LOW);
  digitalWrite(TX_POWER_PIN, LOW);

}

/***************************************************************************
 * i2cSensorOn
***************************************************************************/

void i2cSensorOn(void) {

  digitalWrite(TH1_POWER_PIN, HIGH);  // th
  digitalWrite(LUX_POWER_PIN, HIGH);  // switch on lux module
  digitalWrite(UV_POWER_PIN, HIGH);   // UV module, provision

  delay(100);
  
}

/***************************************************************************
 * i2cSensorOff - DOES NOT WORK PROPERLY!
***************************************************************************/

void i2cSensorOff(void) {

  digitalWrite(UV_POWER_PIN, LOW);    // UV module, provision
  digitalWrite(LUX_POWER_PIN, LOW);   // Lux + UV module
  //digitalWrite(TH1_POWER_PIN, LOW); // block i2c
  
}

/***************************************************************************
 * t1Callback
***************************************************************************/

void t1Callback() {

    i2cSensorOn();
    
    if(HTU21DF_begin() == 0) {
      // read from sensor
      delay(10);
      Temperature = HTU21DF_readTemperature();
      Humidity = HTU21DF_readHumidity();
      HTU21DF_readStatus();
    }

    i2cSensorOff();

    buildAndSendTempHum(Temperature, (int)Humidity, TEMP_HUM_CHNL); 
 
#ifdef _DEBUG_
    Serial.print(F("T1 "));
    Serial.print(millis());     
    Serial.print(F(" T "));        
    Serial.print(Temperature, 1);  
    Serial.print(F(" H "));        
    Serial.print((int)Humidity);   
    //Serial.print(F(" B "));        
    //Serial.print(Battery); 
    Serial.println(); 
#endif

    if (t1.isFirstIteration()) {
      t2.enableDelayed(100); // set space between tx
    }

}

/***************************************************************************
 * t2Callback
***************************************************************************/

void t2Callback() {
  
    // ----------------------------------------
    // LUX High resultion with minimum measurement time
    // ----------------------------------------
    // Extra range for bright light up to 100000 lux
    // The calculation could result into a value up to 120 klx (120000 lux).
    // Normal High Resolution mode, with minimal time measurement 
    // (31 is minimum for MTregister).
    
    i2cSensorOn();
    
    if(BH1750_begin() == 0) {
      BH1750_write8(BH1750_POWER_ON);
      BH1750_write8(BH1750_mt_H(31)); // lowest value of measurement time
      BH1750_write8(BH1750_mt_L(31));
      BH1750_write8(BH1750_ONE_TIME_H_RES_MODE);
      delay(90);            // was 180 * 31 / 69
      lux = (float) BH1750_read16() / BH1750_FACTOR * ( 69.0 / 31.0 ); // BH1750_factor is usually 1.2
      BH1750_write8(BH1750_POWER_DOWN);
    }

    i2cSensorOff();
    
    // read battery value and scale as 0 to 100% being 100% = 3.0V
    // with vref = INTERNAL = 1.1 V = 1024 then 0.00107421875 V/bit (es. 102 = 102 x 0.00107.. = 0,11 v)
    // vbat = adc * 0.0487F taking into account for divider ratio
    vbat = ADC_VBAT_K * analogRead(BATTERY_SENSE); // this is vbat in volts
    vbat = vbat * 100.0F / 3.0F;                   // this is vbat as percentage of nominal 3.0 volts battery
    if (vbat > 99.9) vbat = 99.0;                  // if above 3 volts, limit to 100%
    if (vbat < 0.1) vbat = 1.0;                    // impossible, but just in case
    
    if (lux > 100000) lux = 100000;                // limit lux range to 100 klx
    
    // scale lux and vbat and send as temperature and humidity in -30 to +70 °C range
    buildAndSendTempHum((-50.0F + (lux/1000.0F)), vbat, LUX_BAT_CHNL); 

#ifdef _DEBUG_
    Serial.print(F("T2 "));
    Serial.print(millis());
    Serial.print(F(" L "));  
    Serial.print(lux, 0);    
    Serial.print(F(" V "));
    Serial.print(vbat, 0);
    Serial.println(); 
#endif

    if (t2.isFirstIteration()) {
      t3.enableDelayed(100); // set space between tx
    }
     
}

/***************************************************************************
 * t3Callback
***************************************************************************/

void t3Callback() {

    i2cSensorOn();
    
    if(VEML6070_begin(VEML6070_1_T) == 0) {
      delay(10 + VEML6070_1_T * 90); // wait 10 ms after end of conversion in 90 ms steps
      UVLevel = VEML6070_readUV();
      VEML6070_shutdown();          // Power OFF to save battery, does it work?
    }
    
    i2cSensorOff();

    UVIndex = calcUVI(UVLevel); // Compute UV Index
    
    buildAndSendUV(UVIndex);      // Send UV Index

#ifdef _DEBUG_
    Serial.print(F("T3 "));
    Serial.print(millis()); 
    Serial.print(F(" U ")); 
    Serial.print(UVLevel);
    Serial.print(F(" I ")); 
    Serial.print(UVIndex);  
    Serial.println(); 
#endif
    
}

/***************************************************************************
 * setup
***************************************************************************/

void setup () {

  char cFail = 0;

#ifdef _DEBUG_
  Serial.begin(115200);
  Serial.print(F("MultiLite-168P "));
  Serial.println(SW_VERSION);
#endif

  // configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HW_CFG_PIN, INPUT_PULLUP);
  pinMode(TH1_POWER_PIN, OUTPUT);
  pinMode(TH2_POWER_PIN, OUTPUT);
  pinMode(UV_POWER_PIN, OUTPUT);
  pinMode(LUX_POWER_PIN, OUTPUT);
  pinMode(TX_DATA_PIN, OUTPUT);
  pinMode(TX_POWER_PIN, OUTPUT);

  digitalWrite(TX_DATA_PIN, LOW);
  digitalWrite(TX_POWER_PIN, LOW);
  digitalWrite(TH2_POWER_PIN, LOW);    // Always OFF in Lite version

  // new rolling code with every reset
  randomSeed(analogRead(ANALOG_PIN_1)); // analog pin one used as source of noise for random seed
#ifdef _DEBUG_
  rollingCode = 0x6F; // 111 easier to recognize
#else
  rollingCode = random(0x01, 0xFE);
#endif

  // DEFAULT: the default analog reference of 3.3 volts (on 3.3V Arduino boards)
  // INTERNAL: an built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
  analogReference(INTERNAL);
  // battery read, discard first sample
  vbat = ADC_VBAT_K * analogRead(BATTERY_SENSE); // this is vbat in volts
  delay(100);
  vbat = ADC_VBAT_K * analogRead(BATTERY_SENSE); // this is vbat in volts
  
#ifdef _DEBUG_
  Serial.print(F("Id: "));
  Serial.print(rollingCode); 
  Serial.print(F(" DBG - Bat: "));
  Serial.print(vbat); 
  Serial.println(F(" Volts "));
#endif

  initCrc8();

  Wire.begin();

  i2cSensorOn();

  // Start the sensor and test if the I2C address is right.
  if(HTU21DF_begin() != 0) {
    digitalWrite(TH1_POWER_PIN, LOW);  // switch off TH module  
    cFail += 1;  
  }
  if(BH1750_begin() != 0) {
    digitalWrite(LUX_POWER_PIN, LOW);  // switch off lux module
    cFail = 2; 
  }
  if(VEML6070_begin(VEML6070_1_T) != 0) {
    digitalWrite(UV_POWER_PIN, LOW);  // switch off UV module
    cFail += 4;
  }


  i2cSensorOff(); 
  
  if(cFail == 0) {
      runner.init();
      runner.allowSleep(true);
      runner.addTask(t1);
      runner.addTask(t2);
      runner.addTask(t3);

      // allow powered up items time to ready themselves
      delay(100);
#ifdef _DEBUG_
      Serial.println(F("Run..."));
#endif
      // enable only t1, t2 and t3 are enabled lately from inside t1, t2 to allow 
      // for time spacing of different channels transmissions
      t1.enable();
  }
  else {
    i2cSensorOff();
#ifdef _DEBUG_
    Serial.print(F("!"));
    Serial.println(cFail, DEC);
#endif
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on and off forever
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);  
        delay(90);
    }
  }

}

/***************************************************************************
 * loop
***************************************************************************/

void loop () {
  
  runner.execute();
  
}

/***************************************************************************
 * EOF
***************************************************************************/
