/* This code works with ZMPT101B AC voltage sensor module and 128x32 OLED display
 * It permits you to measure any AC voltage up to 250V, BE CAREFUL !!!
 * The functions from Filters library permits you to calculate the True RMS of a signal
 * Refer to www.surtrTech.com or SurtrTech YouTube channel for more details
 */

#include <Filters.h> //Easy library to do the calculations

float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist

int Sensor; //Sensor analog input, here it's A0

float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage

unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;


void setup() {
  Serial.begin( 9600 );    // start the serial port
}

void loop() {
  
  RunningStatistics inputStats;                //Easy life lines, actual calculation of the RMS requires a load of coding
   
  while( true ) {   
    Sensor = analogRead(A6);  // read the analog in value:
    inputStats.input(Sensor);  // log to Stats function
        
    if((unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();   // update time every second
            
      Serial.print( "\n" );
      
      current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
      current_Volts= current_Volts*(40.3231);                //Further calibrations for the amplitude
      
      Serial.print( "\tVoltage: " );
      Serial.println( current_Volts ); //Calculation and Value display is done the rest is if you're using an OLED display
      
    }
  }
}


/*void setup() {
  Serial.begin(9600);
}

void loop() {
//  Serial.println(analogRead(A0));  // r
//  Serial.println(analogRead(A3));  // s
  Serial.println(analogRead(A6));  // t
//  delay(100);
} */
