#include "EmonLib.h"

// Define digital pinout for drive S0,S1,S2,S3 MUX0
const int S0_0 = 4;
const int S1_0 = 0;
const int S2_0 = 2;
const int S3_0 = 15;
// Define digital pinout for drive S0,S1,S2,S3 MUX1
const int S0_1 = 12;
const int S1_1 = 14;
const int S2_1 = 27;
const int S3_1 = 26;

int64_t oldtime;
int i = 0;

// Factor calibration, get from calculate y=mx+c
//float Fcal = 0.807097582;
//float Fcal = 0.349667607;
//float Fcal = 0.279467369;

char *Fcal0[]={"0.287213466", //ch0
               "0.323757645", //ch1
               "0.311531822", //ch2
               "0.324008188", //ch3
               "0.342384946", //ch4
               "0.325089261", //ch5
               "0.313405141", //ch6
               "0.314034306", //ch7
               "0.338636325", //ch8
               "0.30946127",  //ch9
               "0.276201835", //ch10 *
               "0.317099078", //ch11 *
               "0.322290029", //ch12
               "0.304946751", //ch13
               "0.306132453", //ch14
               "0.319106095"};
               
char *Fcal1[]={"0.309710217", //ch0
               "0.305719647", //ch1
               "0.312504613", //ch2
               "0.31609406",  //ch3
               "0.335830757", //ch4
               "0.321665688", //ch5
               "0.302557889", //ch6
               "0.304467641", //ch7
               "0.337665862", //ch8
               "0.309362523", //ch9
               "0.301756146", //ch10
               "0.310500764", //ch11
               "0.312307403", //ch12 *
               "0.304310657", //ch13
               "0.314151873", //ch14
               "0.312147999"}; 
               
String fc_0;String fc_1;

char *mux[] = {"0000", //ch0 v
               "1000", //ch1 v
               "0100", // ok v
               "1100", // ok v
               "0010", //ch4 v
               "1010", // ok v
               "0110", // ok v
               "1110", //ok v
               "0001", //ok v
               "1001", //ok v
               "0101", //irms0: ok  irms1: ok v
               "1101", //irms0: ok  irms1: ok v
               "0011", //irms0: ok  irms1: ok v
               "1011", //irms0: ok  irms1: ok v
               "0111", //irms0: ok  irms1: ok v
               "1111"}; //irms0: ok  irms1: ok
               
//char *mux[] = {"1111"};

// Define MUX variable             
String ch_mux;
String mux_bit0;
String mux_bit1;
String mux_bit2;
String mux_bit3;

//Emon Init
EnergyMonitor emon0_0;                            // Create an instance emon0
EnergyMonitor emon0_1;                            // Create an instance emon1

void setup() {
  Serial.begin(9600);
  emon0_0.current(A4, 38.3);                       // Current: input pin, calibration. ratio/Rburden
  emon0_1.current(A7, 38.3);                       // Current: input pin, calibration. ratio/Rburden

  // Initialiaze digital pinout for drive S0,S1,S2,S3 MUX0
  pinMode(S0_0, OUTPUT);
  pinMode(S1_0, OUTPUT);
  pinMode(S2_0, OUTPUT);
  pinMode(S3_0, OUTPUT);
  
  // Initialiaze digital pinout for drive S0,S1,S2,S3 MUX1
  pinMode(S0_1, OUTPUT);
  pinMode(S1_1, OUTPUT);
  pinMode(S2_1, OUTPUT);
  pinMode(S3_1, OUTPUT);

  // Initialize value oldtime millis
  oldtime = millis();
}

void loop() {
    if((millis() - oldtime) > 0){
      if((millis() - oldtime) >= 1000){
        oldtime = millis();
        ch_mux = mux[i];
        mux_bit0 = ch_mux.substring(0,1); mux_bit1 = ch_mux.substring(1,2); mux_bit2 = ch_mux.substring(2,3); mux_bit3 = ch_mux.substring(3,4);
//        Serial.println(ch_mux);
//        Serial.print(mux_bit0);Serial.print("|");Serial.print(mux_bit1);Serial.print("|");Serial.print(mux_bit2);Serial.print("|");Serial.println(mux_bit3);
          
        digitalWrite(S0_0, mux_bit0.toInt());digitalWrite(S1_0,mux_bit1.toInt());digitalWrite(S2_0,mux_bit2.toInt());digitalWrite(S3_0,mux_bit3.toInt());
        digitalWrite(S0_1,mux_bit0.toInt());digitalWrite(S1_1,mux_bit1.toInt());digitalWrite(S2_1,mux_bit2.toInt());digitalWrite(S3_1,mux_bit3.toInt()); 

//        fc_0 = Fcal0[i];fc_1 = Fcal1[i];
//        float Irms0_0 = emon0_0.calcIrms(1800) * fc_0.toFloat(); 
//        float Irms0_1 = emon0_1.calcIrms(1800) * fc_1.toFloat(); 

        float Irms0_0 = emon0_0.calcIrms(1800); 
        float Irms0_1 = emon0_1.calcIrms(1800);

//        Serial.print(Fcal0[i]);Serial.print("|");Serial.print(Fcal1[i]);
        Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0);
        Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1); 
        i++;
        if(i>15){
          i =0;
        }
    }
  } 
}
