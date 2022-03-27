#include "EmonLib.h"
#include <WiFi.h>
#include <WiFiUdp.h>

const int statusLED = 13;

const int S0_0 = 4;
const int S1_0 = 0;
const int S2_0 = 2;
const int S3_0 = 15;

const int S0_1 = 12;
const int S1_1 = 14;
const int S2_1 = 27;
const int S3_1 = 26;

float Fcal = 0.807097582;

//Emon Init
EnergyMonitor emon0_0;                            // Create an instance emon0
EnergyMonitor emon0_1;                            // Create an instance emon1

void setup() {
  Serial.begin(115200);
  emon0_0.current(A4, 20);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 47(ex resistor) = 66.
  emon0_1.current(A7, 20);                       // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.


  // by pas MUX
  pinMode(S0_0, OUTPUT);
  pinMode(S1_0, OUTPUT);
  pinMode(S2_0, OUTPUT);
  pinMode(S3_0, OUTPUT);
  
  pinMode(S0_1, OUTPUT);
  pinMode(S1_1, OUTPUT);
  pinMode(S2_1, OUTPUT);
  pinMode(S3_1, OUTPUT);

  // write MUX
  digitalWrite(S0_0, 0);digitalWrite(S1_0, 0);digitalWrite(S2_0, 0);digitalWrite(S3_0, 0);
  digitalWrite(S0_1, 0);digitalWrite(S1_1, 0);digitalWrite(S2_1, 0);digitalWrite(S3_1, 0);
}

void loop() {
  float Irms0_0 = emon0_0.calcIrms(1800) * Fcal;      // Calculate Irms0_0 only 
  float Irms0_1 = emon0_1.calcIrms(1800) * Fcal;      // Calculate Irms0_1 only
  
  Serial.print("|Irms0=");Serial.print(Irms0_0);Serial.print("|Irms1=");Serial.println(Irms0_1);// Irms1
  delay(100);
}
