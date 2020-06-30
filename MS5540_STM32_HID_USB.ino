/*
 MS5540C Miniature Barometer Module
 This program will read your MS5440C or compatible pressure sensor every 5 seconds and show you the calibration words, the calibration factors, 
 the raw values and the compensated values of temperature and pressure.
 Once you read out the calibration factors you can define them in the header of any sketch you write for the sensor.

FOR STM32
*/
#include <String.h>
#include <USBComposite.h>
#define DEFAULT_SERIAL_STRING "00000000001A"
#define PRODUCT "STM32 DAQ BOARD"
#define MANUF "STMicroelectronics"
#define TXSIZE 64
#define RXSIZE 64
#define VID 0x0483
#define PID 0x5750



const uint8_t reportDescription[] = {
      0x06, 0x00, 0xff,              //   USAGE_PAGE (Generic Desktop)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      // System Parameters
    //Прием в мк
      0xa1, 0x01,                    //   COLLECTION (Application)
      0x85, 0x02,                    //   REPORT_ID (2)
      0x75, 0x08,                    //   REPORT_SIZE (8)
      0x95, 64,                        //   REPORT_COUNT (4)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
    //Передача в пк
      0x85, 0x01,                    //   REPORT_ID (1)
      0x09, 0x04,                    //   USAGE (Vendor Usage 4)
      0x75, 0x08,                    //   REPORT_SIZE (8)
      0x95, 64,                        //   REPORT_COUNT (8)
      0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
      0xC0    /*     END_COLLECTION              */
};

USBHIDDevice STM_HID;
uint8 buf_in[RXSIZE];
uint8 buf_out[TXSIZE];
HIDReporter STM_HID_REP(buf_out,TXSIZE);

// include library:
#include <SPI.h>
// calibration factor
long c1,c2,c3,c4,c5,c6;

float TEMP2,PCOMP2;

void toggle_led()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
};


void resetsensor() //this function keeps the sketch a little shorter
{ 
  SPI.setDataMode(SPI_MODE0); 
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);

};

  void calibr()
  {
  resetsensor(); //resets the sensor
  //Calibration word 1
  unsigned int result1 = 0;
  unsigned int inbyte1 = 0;
  SPI.transfer(0x1D); //send first byte of command to get calibration word 1
  SPI.transfer(0x50); //send second byte of command to get calibration word 1
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  result1 = SPI.transfer(0x00); //send dummy byte to read first byte of word
  result1 = result1 << 8; //shift returned byte 
  inbyte1 = SPI.transfer(0x00); //send dummy byte to read second byte of word
  result1 = result1 | inbyte1; //combine first and second byte of word
  resetsensor(); //resets the sensor
  
//Calibration word 2; see comments on calibration word 1
  unsigned int result2 = 0;
  byte inbyte2 = 0; 
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1); 
  result2 = SPI.transfer(0x00);
  result2 = result2 <<8;
  inbyte2 = SPI.transfer(0x00);
  result2 = result2 | inbyte2;
  resetsensor(); //resets the sensor
  
  //Calibration word 3; see comments on calibration word 1
  unsigned int result3 = 0;
  byte inbyte3 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x90); 
  SPI.setDataMode(SPI_MODE1); 
  result3 = SPI.transfer(0x00);
  result3 = result3 <<8;
  inbyte3 = SPI.transfer(0x00);
  result3 = result3 | inbyte3;
  resetsensor(); //resets the sensor

  //Calibration word 4; see comments on calibration word 1
  unsigned int result4 = 0;
  byte inbyte4 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1); 
  result4 = SPI.transfer(0x00);
  result4 = result4 <<8;
  inbyte4 = SPI.transfer(0x00);
  result4 = result4 | inbyte4;
  
  //now we do some bitshifting to extract the calibration factors 
  //out of the calibration words;
   c1 = (result1 >> 1) & 0x7FFF;
   c2 = ((result3 & 0x003F) << 6) | (result4 & 0x003F);
   c3 = (result4 >> 6) & 0x03FF;
   c4 = (result3 >> 6) & 0x03FF;
   c5 = ((result1 & 0x0001) << 10) | ((result2 >> 6) & 0x03FF);
   c6 = result2 & 0x003F;
  resetsensor(); //resets the sensor 
  };

long get_temperature_pressure(long c1,long c2,long c3,long c4,long c5,long c6)
  {
   resetsensor(); 
   //Pressure:
  unsigned int presMSB = 0; //first byte of value
  unsigned int presLSB = 0; //last byte of value
  unsigned int D1 = 0;
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  presMSB = presMSB << 8; //shift first byte
  presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D1 = presMSB | presLSB; //combine first and second byte of value
    

  resetsensor(); //resets the sensor  

  //Temperature:
  unsigned int tempMSB = 0; //first byte of value
  unsigned int tempLSB = 0; //last byte of value
  unsigned int D2 = 0;
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  tempMSB = tempMSB << 8; //shift first byte
  tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D2 = tempMSB | tempLSB; //combine first and second byte of value
 

  //calculation of the real values by means of the calibration factors and the maths
  //in the datasheet. const MUST be long
  const long UT1 = (c5 << 3) + 20224;
  const long dT = D2 - UT1;
  const long TEMP = 200 + ((dT * (c6 + 50)) >> 10);
  const long OFF  = (c2 * 4) + (((c4 - 512) * dT) >> 12);
  const long SENS = c1 + ((c3 * dT) >> 10) + 24576;
  const long X = (SENS * (D1 - 7168) >> 14) - OFF;
  long PCOMP = ((X * 10) >> 5) + 2500;
  
  //2-nd order compensation only for T < 20°C or T > 45°C
  long T2 = 0;
  float P2 = 0;

  if (TEMP < 200)
    {
      T2 = (11 * (c6 + 24) * (200 - TEMP) * (200 - TEMP) ) >> 20;
      P2 = (3 * T2 * (PCOMP - 3500) ) >> 14;
    }
  else if (TEMP > 450)
    {
      T2 = (3 * (c6 + 24) * (450 - TEMP) * (450 - TEMP) ) >> 20;
      P2 = (T2 * (PCOMP - 10000) ) >> 13;    
    }

if ((TEMP > 200) || (TEMP < 450))
  {
 TEMP2 = TEMP - T2;
  PCOMP2 = (PCOMP - P2)*10; // mbar -> Pa
    } 
  }

  void setup() 
    {
    STM_HID.begin(reportDescription, sizeof(reportDescription),PID,VID,MANUF ,PRODUCT ,DEFAULT_SERIAL_STRING);
    SPI.begin(); //see SPI library details on arduino.cc for details
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    pinMode(LED_BUILTIN, OUTPUT);
 
 //Инициализация таймера на частоту 32768 (33 000) Гц, для тактирования 
 // generate a MCKL signal pin
  pinMode(PA1, PWM);
  Timer2.setPrescaleFactor(72); 
  Timer2.setCompare(TIMER_CH2, 15);
  Timer2.setOverflow(30);
  Timer2.refresh();
  Timer2.resume(); 
  delay(100);
    calibr();
};

char buf[50];


void loop() 
{
  
  buf_out[0]=1;
  get_temperature_pressure(c1,c2,c3,c4,c5,c6); //?
  sprintf(buf," P1 %d T1 %d P2 0 T2 0 ",(int)PCOMP2,(int)(TEMP2/10.0));
    for(int i(0);i<strlen(buf);i++)
      buf_out[i]=(uint8_t)buf[i];
       STM_HID_REP.sendReport();
        toggle_led();
}
