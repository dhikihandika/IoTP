#include "EmonLib.h"
#include <Filters.h>                                                              //Easy library to do the calculations
#include <WiFi.h>                                                                 // Include library wifi client (ESP32)
#include <PubSubClient.h>                                                         // Include library Publish and Subscribe MQTT by Nick O'Leary (Knolleary)

#define AvgSize 50                                 // Avg Size for moving avarage

// Emon initialize
EnergyMonitor emon0_0;                             // Create an instance emon0
EnergyMonitor emon0_1;                             // Create an instance emon1

// Define variable ssd, password, & mqtt_broker
const char* ssid = "RF_AP1";                                              
const char* password = "payungteduh123";

const char* mqtt_broker = "172.16.68.72";

WiFiClient espClient;                                                              // Instance name WiFiClient as espClient
PubSubClient client(espClient);                                                    // Instance name PubSubClient as client 

char msgOut[127];                                                                  // Define char_of_array buffer variable publish message
String struv;                                                                      // Define varibale string use as convertion integer to string

// Define digital pinout for drive S0,S1,S2,S3 MUX0 & MUX1
const int S0_0 = 4;
const int S1_0 = 0;
const int S2_0 = 2;
const int S3_0 = 15;
const int S0_1 = 12;
const int S1_1 = 14;
const int S2_1 = 27;
const int S3_1 = 26;

// Factor calibration, get from calculate y=mx+c
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

char *mux[] = {"0000", //ch0
               "1000", //ok
               "0100", //ok
               "1100", //ok
               "0010", //ok
               "1010", //ok
               "0110", //ok
               "1110", //ok
               "0001", //ok
               "1001", //ok
               "0101", //irms0: ok  irms1: ok
               "1101", //irms0: ok  irms1: ok
               "0011", //irms0: ok  irms1: ok
               "1011", //irms0: ok  irms1: ok
               "0111", //irms0: ok  irms1: ok
               "1111"}; //irms0: ok  irms1: ok
               
//char *mux[] = {"0011"};

// Define bit MUX variable            
String ch_mux;
String mux_bit0;
String mux_bit1;
String mux_bit2;
String mux_bit3;

// Filters Library
float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 100.0/testFrequency;     // how long to average the signal, for statistist

float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing

// MvAvg method variable
uint8_t indexR = 0;uint8_t indexS = 0;uint8_t indexT = 0;
float sumR;float sumS;float sumT;
float readingR[AvgSize];float readingS[AvgSize];float readingT[AvgSize];
float MvAvgR;float MvAvgS;float MvAvgT;

// Define variable volatge value
float VoltsR; // VoltageR
float VoltsS; // VoltageS
float VoltsT; // VoltageT

// Define variable millis old time
int64_t oldtime;
int64_t oldsendtime;
int i = 0;
int p = 0;

void setup() {
  Serial.begin(9600);

  setup_wifi();
  client.setServer(mqtt_broker, 1883);
  
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
  oldsendtime = millis();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker!");
      Serial.print("Broker:");
      Serial.println(mqtt_broker);
      client.publish("GMIVCS", "Start Connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
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
        MvAvgR = sumR/AvgSize;
        
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
        MvAvgS = sumS/AvgSize;
        
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
        MvAvgT = sumT/AvgSize;

        // Method to switching MUX
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

            // Publish data to server via MQTT
            struv = "$" + String(i) + "_" +  String(Irms0_0) + ";" +
                    String(i+16) + "_" + String(Irms0_1) + ";" +
                    String(MvAvgR) + ";" +
                    String(MvAvgS) + ";" +
                    String(MvAvgT) + ";";
            struv.toCharArray(msgOut,127);
                  
            Serial.print(" Message publish [");
            Serial.print("GMIVCS");
            Serial.print("]");
            Serial.println(msgOut);
            client.publish("GMIVCS", msgOut);
          
            client.loop(); // This allows the client to maintain the connection and check for any incoming messages                                                               
//            Serial.print("|Irms0_");Serial.print(i);Serial.print(":");Serial.print(Irms0_0);
//            Serial.print("|Irms1_");Serial.print(i+16);Serial.print(":");Serial.print(Irms0_1);
//            Serial.print( "|VRMS_R: " );Serial.print(MvAvgR);
//            Serial.print( "|VRMS_S: " );Serial.print(MvAvgS);
//            Serial.print( "|VRMS_T: " );Serial.println(MvAvgT);

            i++;
            if(i>15){
              i =0;
            }
          }
        } else {
          if((millis() - oldtime) <= 0){
            oldtime=0;  //Reset variable oldtime to zero;  
          }
      }

/*      // Method send data every 30 second
      if((millis() - oldsendtime)>0){
        if((millis() - oldsendtime)>= 30000){
          
        }
      }else{
        if((millis() - oldsendtime)<=0){
          oldsendtime = 0;  //Reset variable oldsendtime to zero;  
        }
      } */
   }
}
