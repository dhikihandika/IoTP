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
char *Fcal0[]={"0.3483915189", //ch0 
               "0.334741597", //ch1 !
               "0.2481273182", //ch2 
               "0.2443914471", //ch3 !
               "0.3390481444", //ch4 !
               "0.3406978032", //ch5
               "0.154967303",  //ch6 !
               "0.1814582319", //ch7 !
               "0.2616987967", //ch8 !
               "0.2651833511", //ch9 !
               "0.2665140304", //ch10 
               "0.2806773577", //ch11 !
               "0.2555838578", //ch12 !
               "0.341920943",  //ch13
               "0.3299479238", //ch14 !
               "0.3401528792"};//ch15 !
               
char *Fcal1[]={"0.3464943087", //ch16
               "0.261896624",  //ch17 !
               "0.3405796306", //ch18
               "0.3350948335", //ch19 !
               "0.2546294282", //ch20 !
               "0.3097536734", //ch21
               "0.3191304408", //ch22 !
               "0.2504305674", //ch23 !
               "0.2605842044", //ch24
               "0.3403125822", //ch25
               "0.3411271443", //ch26
               "0.2502868476", //ch27
               "0.2899215104", //ch28 
               "0.3129363368", //ch29
               "0.2946408097", //ch30
               "0.2811357175"}; //ch31
               
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

        fc_0 = Fcal0[i];fc_1 = Fcal1[i];
        float Irms0_0 = emon0_0.calcIrms(1800) * fc_0.toFloat(); 
        float Irms0_1 = emon0_1.calcIrms(1800) * fc_1.toFloat(); 

//        float Irms0_0 = emon0_0.calcIrms(1800); 
//        float Irms0_1 = emon0_1.calcIrms(1800);

        Serial.print(Fcal0[i]);Serial.print("|");Serial.print(Fcal1[i]);
        Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0);
        Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1); 
        i++;
        if(i>15){
          i =0;
        }
    }
  } 
}
