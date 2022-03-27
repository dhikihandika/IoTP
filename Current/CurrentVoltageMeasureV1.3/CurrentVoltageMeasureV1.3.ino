#include "EmonLib.h"
#include <Filters.h> //Easy library to do the calculations
#include <ArduinoMqttClient.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>


#define DEBUG
#define RELEASE

#define AvgSize 50                                 // Avg Size for moving avarage

const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "172.16.68.72";
int        port     = 1883;
const char topic[]  = "wheel1/power_meter/esp8266";
const char topic2[]  = "real_unique_topic_2";
const char topic3[]  = "real_unique_topic_3";

//WiFi config
IPAddress local_IP(172, 16, 68, 201);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);


// Emon initialize
EnergyMonitor emon0_0;                             // Create an instance emon0
EnergyMonitor emon0_1;                             // Create an instance emon1

// Factor calibration, get from calculate y=mx+c
//float Fcal = 0.807097582;
//float Fcal = 0.349667607;
float Fcal = 0.279467369;

//Define sensor_id
const int sensor_idv = 1;

int calibration = 1;

// Define digital pinout for drive S0,S1,S2,S3 MUX0 & MUX1
const int S0_0 = 4;
const int S1_0 = 0;
const int S2_0 = 2;
const int S3_0 = 15;
const int S0_1 = 12;
const int S1_1 = 14;
const int S2_1 = 27;
const int S3_1 = 26;

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

void setup() {
  Serial.begin(9600);
  emon0_0.current(D4, 38.3);                       // Current: input pin, calibration. ratio/Rburden
  emon0_1.current(D7, 38.3);                       // Current: input pin, calibration. ratio/Rburden

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

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //WiFi init
  Serial.print("Connecting");
  Serial.println();
  while (WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print("=");
  }
  Serial.println("");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  if (WiFi.status() == WL_CONNECTED) { //WiFi connected
    Serial.println("Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  //Upload Program Over The Air (OTA)
  ArduinoOTA.setPassword((const char*) "h.123+Gmi8");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize value oldtime millis
  oldtime = millis();
  oldsendtime = millis();
}

void loop() {
    // call poll() regularly to allow the library to send MQTT keep alive which
    // avoids being disconnected by the broker
    mqttClient.poll();
    ArduinoOTA.handle();
    RunningStatistics inputStatsR;                        //Easy life lines, actual calculation of the RMS requires a load of coding
    RunningStatistics inputStatsS;  
    RunningStatistics inputStatsT;  
    inputStatsR.setWindowSecs(windowLength);
    inputStatsS.setWindowSecs(windowLength);
    inputStatsT.setWindowSecs(windowLength);
    
    while(true) {   
      inputStatsR.input(analogRead(D0));                  // log to Stats function
      inputStatsS.input(analogRead(D3));                  // log to Stats function
      inputStatsT.input(analogRead(D6));                  // log to Stats function
      
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
            
            float Irms0_0 = emon0_0.calcIrms(1800)*Fcal; 
            float Irms0_1 = emon0_1.calcIrms(1800)*Fcal; 
               
//            Serial.print("|Irms0_");Serial.print(i);Serial.print(":");Serial.print(Irms0_0);
//            Serial.print("|Irms1_");Serial.print(i+16);Serial.print(":");Serial.print(Irms0_1);
//            Serial.print( "|VRMS_R: " );Serial.print(MvAvgR);
//            Serial.print( "|VRMS_S: " );Serial.print(MvAvgS);
//            Serial.print( "|VRMS_T: " );Serial.println(MvAvgT);

            String _sendData1 = "[{\"sensor_id\":\"";
            _sendData1 += sensor_idv;
            _sendData1 += "\",\"voltage_r\":\"";
            _sendData1 += MvAvgR;
            _sendData1 += "\",\"voltage_s\":\"";
            _sendData1 += MvAvgS;
            _sendData1 += "\",\"voltage_t\":\"";
            _sendData1 += MvAvgT;
        
            _sendData1 += "\"},{\"sensor_id\":\"";
            _sendData1 += i;
            _sendData1 += "\",\"current_mx0\":\"";
            _sendData1 += Irms0_0;
            
            _sendData1 += "\"},{\"sensor_id\":\"";
            _sendData1 += i+16;
            _sendData1 += "\",\"current_mx1\":\"";
            _sendData1 += Irms0_1;
        
//          Serial.println(_sendData1);
            char dataBuffer1[2048];
            _sendData1.toCharArray(dataBuffer1, 2048);
            if (calibration < 5) {
              Serial.print("Calibration=> ");
              Serial.println(calibration);
              calibration++;
            } else {
              mqttClient.beginMessage(topic);
              mqttClient.printf(dataBuffer1);
              mqttClient.endMessage();
              Serial.println("Data1 sent!");
              digitalWrite(13, HIGH);
              delay(200);
              digitalWrite(13, LOW);
              delay(200);
            }

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
