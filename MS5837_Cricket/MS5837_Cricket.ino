/* 07/01/2019 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 The MS5837-30BA is a new generation of  high resolution pressure sensors with I2C bus interface for     
 depth measurement systems with a water depth resolution of 2  mm. The sensor module includes a high linearity  
 pressure sensor and an ultra-lowpower 24 bit ΔΣADC with internal factory calibrated coefficients.  It provides  
 a precise digital 24 Bit pressure and temperature value and different operation modes that allow the user to 
 optimize for conversion speed and current consumption.  A high resolution temperature output allows the 
 implementation in depth measurement systems and thermometer function without any additional sensor.   
 The MS5837-30BA can be interfaced to virtually any microcontroller. The communication protocol is simple, 
 without the need of programming internal registers in the device.  The gel protection and antimagnetic stainless
 steel cap make the module water resistant.  

 http://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf
 
 Library may be used freely and without limit with attribution.
 
*/
#include <STM32L0.h>
#include <RTC.h>
#include "MS5837.h"
#include "I2Cdev.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// Cricket pin assignments
#define myLed     10 // blue led 
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

// RTC set up
/* Change these values to set the current initial time */

uint8_t seconds = 0;
uint8_t minutes = 20;
uint8_t hours = 13;

/* Change these values to set the current initial date */

uint8_t day =   5;
uint8_t month = 5;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = true;


bool SerialDebug = true;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp;

// MS5837 configuration

// Specify sensor full scale
uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate

uint16_t Pcal[8];         // calibration constants from MS5837 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5837 pressure and temperature data
double dT, OFFSET, SENS, TT2, OFFSET2, SENS2;  // First order and second order corrections for raw MS5837 temperature and pressure data
    
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5837 pressures sensor pressure and temperature
float fluidDensity = 1029.0f; // kg/m^3 for seawater

MS5837 MS5837(&i2c_0); // instantiate MS5837 class

void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(myVBat_en, OUTPUT);
  pinMode(myVBat, INPUT);
  analogReadResolution(12);

  pinMode(A3, INPUT);
  
  I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                                      // should detect MS5837 at 0x76
  delay(1000);


  // Set the RTC time
  SetDefaultRTC();

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  digitalWrite(myVBat_en, HIGH);
  VBAT = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
  digitalWrite(myVBat_en, LOW);
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");
  
  // Reset the MS5837 pressure sensor
  MS5837.Reset();
  delay(100);
  Serial.println("MS5837 pressure sensor reset...");
  // Read PROM data from MS5837 pressure sensor
  MS5837.PromRead(Pcal);
  Serial.println("PROM dta read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  
  nCRC = MS5837.checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5837 calibration data
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
  
   // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);  
  
  /* end of setup */

}
/* 
 *  
 * Everything in the main loop is based on interrupts, so that 
 * if there has not been an interrupt event the STM32L082 should be in STOP mode
*/
 
void loop()
{
  /*RTC*/
  if (alarmFlag) { // update serial output and log to SPI flash whenever there is an RTC alarm
      alarmFlag = false;
      
    D1 = MS5837.DataRead(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5837.DataRead(ADC_D2, OSR);  // get raw temperature value
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 16) + dT*Pcal[4]/pow(2,7);
    SENS = Pcal[1]*pow(2,15) + dT*Pcal[3]/pow(2,8);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      TT2 = 2*dT*dT/pow(2, 37); // correction for high temperatures
      OFFSET2 = 1*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      TT2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 3*(100*Temperature - 2000)*(100*Temperature - 2000)/2;
      SENS2   = 5*(100*Temperature - 2000)*(100*Temperature - 2000)/8;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 7*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 4*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 
     Temperature = Temperature - TT2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 13))/10;  // Pressure in mbar or hPa

     float altitude = 145366.45f*(1. - pow((Pressure/1013.25f), 0.190284f));
     // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
     float depth = (Pressure*100.0f - 101300.0f)/(fluidDensity*9.80665f);
   
     if(SerialDebug) {
     Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
     Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
     Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
     Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
     Serial.print("Depth = "); Serial.print(depth, 2); Serial.println(" meters");
    }   
    
      VDDA = STM32L0.getVDDA();
      digitalWrite(myVBat_en, HIGH);
      VBAT = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
      digitalWrite(myVBat_en, LOW);
      
      if(SerialDebug) {
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    }
    
    // Read RTC
    Serial.println("RTC:");
    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds);

    Serial.print("RTC Time = ");
    if (hours < 10)   {Serial.print("0");Serial.print(hours); } else Serial.print(hours);
    Serial.print(":");
    if (minutes < 10) {Serial.print("0"); Serial.print(minutes); } else Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {Serial.print("0"); Serial.print(seconds); } else Serial.print(seconds);
    Serial.println(" ");

    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();
      
    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);
        
    } // end of alarm section
    
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */

void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}

void SetDefaultRTC()                                                                                 // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}



