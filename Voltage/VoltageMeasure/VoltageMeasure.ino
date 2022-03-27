/* This code works with ZMPT101B AC voltage sensor module and 128x32 OLED display
 * It permits you to measure any AC voltage up to 250V, BE CAREFUL !!!
 * The functions from Filters library permits you to calculate the True RMS of a signal
 * Refer to www.surtrTech.com or SurtrTech YouTube channel for more details
 */

#include <Filters.h> //Easy library to do the calculations

//MvAvg method
#define AvgSize 50
uint8_t indexR = 0;uint8_t indexS = 0;uint8_t indexT = 0;
float sumR;float sumS;float sumT;
float readingR[AvgSize];float readingS[AvgSize];float readingT[AvgSize];
float MvAvgR;float MvAvgS;float MvAvgT;

float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 100.0/testFrequency;     // how long to average the signal, for statistist

float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float VoltsR; // VoltageR
float VoltsS; // VoltageS
float VoltsT; // VoltageT

unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;

void setup() {
  Serial.begin( 9600 );    // start the serial port
}

void loop() {
  RunningStatistics inputStatsR;                        //Easy life lines, actual calculation of the RMS requires a load of coding
  RunningStatistics inputStatsS;  
  RunningStatistics inputStatsT;  
  inputStatsR.setWindowSecs(windowLength);
  inputStatsS.setWindowSecs(windowLength);
  inputStatsT.setWindowSecs(windowLength);
  
  while(true) {   
    inputStatsR.input(analogRead(A0));                  // log to Stats function
    inputStatsS.input(analogRead(A3));                  // log to Stats function
    inputStatsT.input(analogRead(A6));                  // log to Stats function
    
      VoltsR = intercept + slope * inputStatsR.sigma(); // Calibartions for offset and amplitude
      VoltsR= VoltsR*(40.3231);                         // Further calibrations for the amplitude
      // movingAvgR method
      sumR = sumR - readingR[indexR];
      readingR[indexR] = VoltsR;
      sumR = sumR + readingR[indexR];
      indexR = indexR + 1;
      if(indexR >= AvgSize){
         indexR = 0;
      }
      
      VoltsS = intercept + slope * inputStatsS.sigma(); // Calibartions for offset and amplitude
      VoltsS= VoltsS*(40.3231);                         // Further calibrations for the amplitude
      // movingAvgS method
      sumS = sumS - readingS[indexS];
      readingS[indexS] = VoltsS;
      sumS = sumS + readingS[indexS];
      indexS = indexS + 1;
      if(indexS >= AvgSize){
         indexS = 0;
      }
      
      VoltsT = intercept + slope * inputStatsT.sigma(); // Calibartions for offset and amplitude
      VoltsT= VoltsT*(40.3231);                         // Further calibrations for the amplitude
      // movingAvgR method
      sumT = sumT - readingT[indexT];
      readingT[indexT] = VoltsT;
      sumT = sumT + readingT[indexT];
      indexT = indexT + 1;
      if(indexT >= AvgSize){
         indexT = 0;
      }
    
    if((unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();   // update time every second
      
      MvAvgR = sumR/AvgSize;
      MvAvgS = sumS/AvgSize;
      MvAvgT = sumT/AvgSize;
      Serial.print( "VoltageR: " );Serial.print(MvAvgR);Serial.print("|");
      Serial.print( "VoltageS: " );Serial.print(MvAvgS);Serial.print("|");
      Serial.print( "VoltageT: " );Serial.println(MvAvgT);
    }
  }
}
