/*
 MS5540C Miniature Barometer Module
 This program will read your MS5440C or compatible pressure sensor every 5 seconds and show you the calibration words, the calibration factors, 
 the raw values and the compensated values of temperature and pressure.
 Once you read out the calibration factors you can define them in the header of any sketch you write for the sensor.

FOR STM32


 Calibration of my sensor example ...
 
 Calibration word 1 = 46958
 Calibration word 2 = 65369
 Calibration word 3 = 39392
 Calibration word 4 = 45914
 
 c1 = 23479
 c2 = 2074
 c3 = 717
 c4 = 615
 c5 = 1021
 c6 = 25
*/

#define debug 1
// include library:
#include <SPI.h>
#include <usb_serial.h>
  USBSerial myUserial;
  
float Temp,Pressure;

void toggle_led()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
};

long c1,c2,c3,c4,c5,c6;
float TEMP2 ,PCOMP2;
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
 
    if(debug)
    {
         myUserial.print("Calibration word 1 = ");
          myUserial.print(result1,HEX);
           myUserial.print(" ");  
            myUserial.println(result1);
  };
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
    if(debug==1)
    {
        myUserial.print("Calibration word 2 = ");
          myUserial.print(result2,HEX);  
            myUserial.print(" ");  
              myUserial.println(result2);  
    };
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
    if(debug)
    {
        myUserial.print("Calibration word 3 = ");
          myUserial.print(result3,HEX);  
            myUserial.print(" ");  
              myUserial.println(result3);  
                     };
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
    if(debug)
    {
            myUserial.print("Calibration word 4 = ");
              myUserial.print(result4,HEX);
                myUserial.print(" ");  
                  myUserial.println(result4);  
                    };  
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

  float get_temperature_pressure(long c1,long c2,long c3,long c4,long c5,long c6)
  {

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
    
    if(debug)
    {
      myUserial.print("D1 - Pressure raw = ");
        myUserial.println(D1);
                      };

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
 
      if(debug)
    {
      myUserial.print("D2 - Temperature raw = ");
        myUserial.println(D2); //voila!
                    };
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

if ((TEMP > 200) or (TEMP < 450))
  {
  //  TEMP2 = TEMP - T2;
  //  PCOMP2 = (PCOMP - P2)*10;
   
  } 
  else{
    TEMP2 = TEMP - T2;
     PCOMP2 = (PCOMP - P2)*10;     
    };  
  }

void  setup()
   {
    myUserial.begin();
    SPI.begin(); //see SPI library details on arduino.cc for details
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    pinMode(LED_BUILTIN, OUTPUT);

 //Инициализация таймера на частоту 32768 Гц, для тактирования 
 // generate a MCKL signal pin
  pinMode(PA1, PWM);
  pinMode(PA2, PWM);
   Timer2.setPrescaleFactor(72); // 1 µs resolution
   Timer2.setCompare(TIMER_CH2, 15);
   Timer2.setCompare(TIMER_CH3, 15);
   Timer2.setOverflow(30);
   Timer2.refresh();
   Timer2.resume(); 

  
  delay(100);
};
char buf[50];

void loop() 
{

  
  // Write main calibration factor
  calibr();                     
  get_temperature_pressure(c1,c2,c3,c4,c5,c6); //?
  Temp = TEMP2/10.0;
  Pressure = PCOMP2; 
  sprintf(buf,"P1 %d T1 %d P2 0 T2 0 \n",(int)Pressure,(int)Temp);
 
    myUserial.write(buf);
        toggle_led();
           delay(1000);
}
