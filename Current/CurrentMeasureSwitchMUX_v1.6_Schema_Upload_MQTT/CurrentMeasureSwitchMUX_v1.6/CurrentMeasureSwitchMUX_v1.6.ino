#include "EmonLib.h"
#include <ArduinoMqttClient.h>
#include <WiFi.h>

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
int cal_literation = 1;
int p = 0;int i = 0;int j = 0;

int sensor_idv = 1;
float MvAvgR = 221;
float MvAvgS = 221;
float MvAvgT = 221;

float y_irms[32];
float Irms0[16];
float Irms1[16];
               
String fc_0;String fc_1;

//WiFi config
IPAddress local_IP(172, 16, 68, 201);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// MQTT config
const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "172.16.68.72";
int        port     = 1883;
const char topic[]  = "wheel1/power_meter/esp32/send";
const char topic2[] = "real_unique_topic_2";
const char topic3[] = "real_unique_topic_3";

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
               "1111"}; //irms0: ok  irms1: ok v
               
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

  // Initialiaze connection to WiFi and MQTT
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
  mqttClient.poll();
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
            _sendData1 += "\",\"current_0\":\"";
            _sendData1 += Irms0_0;
        
            
            _sendData1 += "\"},{\"sensor_id\":\"";
            _sendData1 += i+16;
            _sendData1 += "\",\"current_1\":\"";
            _sendData1 += Irms0_1;
            _sendData1 += "\"}]";
        
//          Serial.println(_sendData1);
            char dataBuffer1[2048];
            _sendData1.toCharArray(dataBuffer1, 2048);
            mqttClient.beginMessage(topic);
            mqttClient.printf(dataBuffer1);
            mqttClient.endMessage();
            Serial.println("Data1 sent!");
            digitalWrite(13, HIGH);
            delay(200);
            digitalWrite(13, LOW);
            delay(200);
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
