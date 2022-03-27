#include "EmonLib.h"

//#define DEBUG

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
int cal_literation = 19;
int p = 0;int i = 0;int j = 0;

float y_irms[32];
float Irms0[16];
float Irms1[16];
               
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

bool is_calibrate = false;

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
  // Initialiaze digital pinout for LED
  pinMode(13, OUTPUT);
  
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
  is_calibrate = false;
  initiate_data();
  Serial.println("START");
  oldtime = millis();
}

void initiate_data(){
  for (int m = 0; m <= 31; m++){
    y_irms[m]=0.67;

    #ifdef DEBUG
    Serial.print("y_irms");Serial.print(m);Serial.print(":");Serial.println(y_irms[m]);
    #endif
  }
  for (int n = 0; n <= 15; n++){
    Irms0[n]= 0;
    Irms1[n]= 0;
  }
}

void blink_led(){
  for (int o = 0; o <= 1; o++){
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}

void loop() {
  
    if((millis() - oldtime) > 0){
      if((millis() - oldtime) >= 1000){
        oldtime = millis();
        ch_mux = mux[i];
        mux_bit0 = ch_mux.substring(0,1); mux_bit1 = ch_mux.substring(1,2); mux_bit2 = ch_mux.substring(2,3); mux_bit3 = ch_mux.substring(3,4);
//        Serial.println(ch_mux);
//        Serial.print(mux_bit0);Serial.print("|");Serial.print(mux_bit1);Serial.print("|");Serial.print(mux_bit2);Serial.print("|");Serial.println(mux_bit3);
          
        digitalWrite(S0_0,mux_bit0.toInt());digitalWrite(S1_0,mux_bit1.toInt());digitalWrite(S2_0,mux_bit2.toInt());digitalWrite(S3_0,mux_bit3.toInt());
        digitalWrite(S0_1,mux_bit0.toInt());digitalWrite(S1_1,mux_bit1.toInt());digitalWrite(S2_1,mux_bit2.toInt());digitalWrite(S3_1,mux_bit3.toInt()); 
        
        if(is_calibrate == true){
          fc_0 = Irms0[i];fc_1 = Irms1[i];
          float Irms0_0 = emon0_0.calcIrms(1800) * fc_0.toFloat(); 
          float Irms0_1 = emon0_1.calcIrms(1800) * fc_1.toFloat(); 
          
          Serial.print(Irms0[i]);Serial.print("|");Serial.print(Irms1[i]);
          Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0,5);
          Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1,5);
        }
        
        if(is_calibrate == false){
          float Irms0_0 = emon0_0.calcIrms(1800); 
          float Irms0_1 = emon0_1.calcIrms(1800);
        
          Irms0[j] += y_irms[j]/Irms0_0;
          Irms1[j] += y_irms[j+15]/Irms0_1; 

          #ifdef DEBUG
          Serial.print("M0:");Serial.print(Irms0[j],5);Serial.print("|");Serial.print("M1:");Serial.println(Irms1[j],5); 
          Serial.print(p);Serial.print("|");Serial.println(is_calibrate);
          Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0,5);
          Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1,5);
          #endif
          
          if((p==cal_literation)&&(i>=15)){
            is_calibrate = true;
            for (int k = 0; k <= 15; k++){
              Irms0[k]=Irms0[k]/(cal_literation+1);
              Irms1[k]=Irms1[k]/(cal_literation+1);
              
              Serial.print("Avg_M");Serial.print(k);Serial.print(":");Serial.print(Irms0[k],5);Serial.print("|");
              Serial.print("Avg_M");Serial.print(k+16);Serial.print(":");Serial.println(Irms1[k],5);
            } 
            blink_led();
          }     
        }
        
        i++;
        j=i;        
        if(i>15){
          i = 0;
          if(is_calibrate == false){
            j=i;
            p++;
          }
        }
    }
  } 
  
}
