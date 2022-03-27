//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//-------------------------------                                     Current Sensor Measure Module                                      ----------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//

#include "EmonLib.h"                            //Include Emon Library 
EnergyMonitor emon0;                            // Create an instance emon0
EnergyMonitor emon1;                            // Create an instance emon1
EnergyMonitor emon2;                            // Create an instance emon2
EnergyMonitor emon3;                            // Create an instance emon3
EnergyMonitor emon4;                            // Create an instance emon4
EnergyMonitor emon5;                            // Create an instance emon5
EnergyMonitor emon6;                            // Create an instance emon6
EnergyMonitor emon7;                            // Create an instance emon7
EnergyMonitor emon8;                            // Create an instance emon8

void setup() {
    Serial.begin(9600);                         // Serial begin initialization
    emon0.current(0, 66);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 47(ex resistor) = 66.
    emon1.current(1, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon2.current(2, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon3.current(3, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon4.current(4, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon5.current(5, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon6.current(6, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon7.current(7, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon8.current(8, 47);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
 } 

void loop() {
    float Irms0 = emon0.calcIrms(1800);      // Calculate Irms0 only 
    float Irms1 = emon1.calcIrms(1800);      // Calculate Irms1 only   
    float Irms2 = emon2.calcIrms(1800);      // Calculate Irms2 only   
    float Irms3 = emon3.calcIrms(1800);      // Calculate Irms3 only   
    float Irms4 = emon4.calcIrms(1800);      // Calculate Irms4 only   
    float Irms5 = emon5.calcIrms(1800);      // Calculate Irms5 only   
    float Irms6 = emon6.calcIrms(1800);      // Calculate Irms6 only   
    float Irms7 = emon7.calcIrms(1800);      // Calculate Irms7 only   
    float Irms8 = emon8.calcIrms(1800);      // Calculate Irms8 only      

    Serial.print("|Irms0=");Serial.print(Irms0);// Irms0  
    Serial.print("|Irms1=");Serial.print(Irms1);// Irms1 
    Serial.print("|Irms2=");Serial.print(Irms2);// Irms2 
    Serial.print("|Irms3=");Serial.print(Irms3);// Irms3 
    Serial.print("|Irms4=");Serial.print(Irms4);// Irms4 
    Serial.print("|Irms5=");Serial.print(Irms5);// Irms5 
    Serial.print("|Irms6=");Serial.print(Irms6);// Irms6 
    Serial.print("|Irms7=");Serial.print(Irms7);// Irms7 
    Serial.print("|Irms8=");Serial.println(Irms8);// Irms8  
    Serial.println("----------------------------------------------------------------------------------------");
    delay(100);
}
