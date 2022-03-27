#include "EmonLib.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>

#define DEBUG
#define CAL_LITERATION 19 //literation for calibration

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

const char broker[]         = "172.16.68.72";
int        port             = 1883;
const char topicPublish[]   = "wheel1/power_meter/data/subs1";
const char topicPublish2[]  = "wheel1/power_meter/calibration/subs1";
const char topicSubscribe[] = "wheel1/power_meter/calibration/pubs1";

unsigned long payloadSize = 268435456;
bool retain = false;
int Qos = 1;
bool dubplicat = false;

// Multiplexer CD74HC4067 config
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

// Truth tabel implementation of CD74HC4067 multiplexer
//              s0, s1, s2, s3    en  channelActive
char *mux[] = {"0   0   0   0", //0   ch0 
               "1   0   0   0", //0   ch1 
               "0   1   0   0", //0   ch2 
               "1   1   0   0", //0   ch3
               "0   0   1   0", //0   ch4 
               "1   0   1   0", //0   ch5
               "0   1   1   0", //0   ch6
               "1   1   1   0", //0   ch7
               "0   0   0   1", //0   ch8
               "1   0   0   1", //0   ch9
               "0   1   0   1", //0   ch10
               "1   1   0   1", //0   ch11
               "0   0   1   1", //0   ch12
               "1   0   1   1", //0   ch13
               "0   1   1   1", //0   ch14
               "1   1   1   1"};//0   ch15

// Define MUX variable             
String ch_mux;
String mux_bit0;
String mux_bit1;
String mux_bit2;
String mux_bit3;

// Buffer, parse and calibration_instruction variable
String dataIn = "";
String dt[33];
int instruction_calibrate = 33; //no instruction
bool is_calibrate;
bool close_doorBuffer;

int r,s,t,u = 0; //for parsing data
int a,b,c,d = 0; //for send data VC
int i,j,k,l,m,n,o,p,q = 0; // for auto calibration and send data Cal

int board_id = 1;
float MvAvgR = 221;
float MvAvgS = 221;
float MvAvgT = 221;

float Irms0_0[16];
float Irms0_1[16];
float Irms_tot[33];
float y_irms[32];                                                             // buffer variable for constant calibartion (y)
float m_irms0[16];                                                            // buffer variable for factor calibration mux0 (m)
float m_irms1[16];                                                            // buffer variable for factor calibration mux1 (m)             
String fc_0;String fc_1;                                                      // buffer string variable for factor calibration mux1 (m)

// Buffer old millis time
int64_t oldtime;

// Emon Init
EnergyMonitor emon0_0;                            // Create an instance emon0
EnergyMonitor emon0_1;                            // Create an instance emon1

//======================================= INITIAILZE PROGRAM (SETUP) =================================================//
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

  // Initialiaze connection to WiFi, MQTT and Schema MQTT Subscribtion Message 
  wifiSetup();
  MQTTconnection();
  mqttClient.poll();
  
  is_calibrate = false; 
  close_doorBuffer = false;  
//  avb_dataInBuffer = false;
  
  // Initialize value oldtime millis
  Serial.println("START IoT DEVICE !!!");
  oldtime = millis();
}

//======================================= FUNCTION & PROCEDURE =================================================//
void wifiSetup(){
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //WiFi init
  Serial.print("Connecting");
  Serial.println();
  wifiHandling();
}

void wifiHandling(){
  while (WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print(".");
  }
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  if (WiFi.status() == WL_CONNECTED) { //WiFi connected
    Serial.println("");
    Serial.print("Wifi Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void MQTTconnection(){
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  MQTThandling();
}

void MQTThandling(){ // handling renew connection MQTT Client "ESP32" to Broker
  if(!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }
}

void subsMQTTMessage(){
  if(!mqttClient.connected()){
    MQTThandling();
  }else{
    mqttClient.onMessage(onMqttMessage);   // Set the message receive callback
    Serial.print("Subscribing to topic: ");
    Serial.println(topicSubscribe);
    Serial.println();
          
    mqttClient.subscribe(topicSubscribe, Qos); // subscribe to a topic 
          
    Serial.print("Waiting for messages on topic: ");
    Serial.println(topicSubscribe);
    Serial.println();
  }
}

void onMqttMessage(int messageSize) {
  // use the Stream interface to print the contents
  if(mqttClient.available()) {
//    Serial.print((char)mqttClient.read());
    dataIn = mqttClient.readString();
    parse_data();
    dataIn="";  
    close_doorBuffer = true;
  }
  
  #ifdef DEBUG
  Serial.println();
  #endif
}

void parse_data(){
  r=0;
  
  #ifdef DEBUG
  Serial.print("in_data: ");
  Serial.println(dataIn);
  #endif
   
  // Initialize variabel, (variable reset)
  for (s = 0; s <= 31; s++){
    dt[s]="";
  }
  // Process parse data
  for(t = 1; t<dataIn.length(); t++){     // Character check every char data
    if((dataIn[t] == '$') || (dataIn[t] == ';')){    // Increment r, use for change index array buffer
    r++;
    dt[r]="";       // Initialize array dt[j]
    }else{
    dt[r] = dt[r] + dataIn[t];     // Process tampung data
    }
  }
  
  // Save data parse to valiable dloat array 'y_irms[u]'
  instruction_calibrate = dt[0].toInt();
  
  #ifdef DEBUG
  Serial.print("status:");Serial.println(dt[0].toInt());
  #endif
  
  if(instruction_calibrate == 0){ // NO instruction to calibrate
    for (u = 0; u <= 15; u++){
      m_irms0[u] = dt[u+1].toFloat();
      m_irms1[u] = dt[u+17].toFloat();
      
      #ifdef DEBUG
      Serial.print(m_irms0[u],10); Serial.print("|"); Serial.println(m_irms1[u],10);
      #endif
    }
  } else {
    if(instruction_calibrate == 1){ // YES instruction to calibrate
      for (u = 0; u <= 31; u++){
        y_irms[u] = dt[u+1].toFloat();
        
        #ifdef DEBUG
        Serial.println(y_irms[u]);
        #endif
      }
      #ifdef DEBUG
      Serial.println("start calibration!");
      #endif
    }
  }
}

void blink_led1(){ // Indicator for publish data VC
    digitalWrite(13, HIGH);delay(200);
    digitalWrite(13, LOW);delay(200);
}

//========================================== MAIN LOOP  =================================================//
void loop() {
  // Call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  if(close_doorBuffer == false){  // Check would doorBuffer is open
    subsMQTTMessage();
  } else {
    if(close_doorBuffer == true){
      mqttClient.unsubscribe(topicSubscribe);   // unsubscribed to a topic 

      if(!mqttClient.connected()){ // handling MQTT connection when autocalibratin proces run
        MQTThandling();
      } else {
        if((millis() - oldtime) > 0){
          if((millis() - oldtime) >= 1000){
            oldtime = millis();
            ch_mux = mux[i];
            mux_bit0 = ch_mux.substring(0,1); mux_bit1 = ch_mux.substring(1,2); mux_bit2 = ch_mux.substring(2,3); mux_bit3 = ch_mux.substring(3,4);
            
            #ifdef DEBUG
            Serial.println(ch_mux);
            Serial.print(mux_bit0);Serial.print("|");Serial.print(mux_bit1);Serial.print("|");Serial.print(mux_bit2);Serial.print("|");Serial.println(mux_bit3);
            #endif
               
            digitalWrite(S0_0,mux_bit0.toInt());digitalWrite(S1_0,mux_bit1.toInt());digitalWrite(S2_0,mux_bit2.toInt());digitalWrite(S3_0,mux_bit3.toInt());
            digitalWrite(S0_1,mux_bit0.toInt());digitalWrite(S1_1,mux_bit1.toInt());digitalWrite(S2_1,mux_bit2.toInt());digitalWrite(S3_1,mux_bit3.toInt()); 
  
            // Scema read data and send every 15 seconds
            if(((is_calibrate == true) && (instruction_calibrate == 1)) || ((is_calibrate == false) && (instruction_calibrate == 0))){
              digitalWrite(13,LOW); // on led indicator for calibarion process
              fc_0 = m_irms0[i];fc_1 = m_irms1[i];
              Irms0_0[i] = emon0_0.calcIrms(1800) * fc_0.toFloat(); 
              Irms0_1[i] = emon0_1.calcIrms(1800) * fc_1.toFloat(); 
  
              #ifdef DEBUG
              Serial.print(m_irms0[i]);Serial.print("|");Serial.print(m_irms1[i]);
              Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0[i],5);
              Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1[i],5);
              #endif
              
              if(i>=15){
                for(a = 0; a <= 15; a++){
                  Irms_tot[a] = Irms0_0[a];
                  Irms_tot[a+16] = Irms0_1[a];
                }
                for(b = 0; b <= 10; b++){
                  String _sendData1 = "[{\"dataVC\":\"";
                  _sendData1 += board_id;
                  _sendData1 += ";";
                  _sendData1 += MvAvgR;
                  _sendData1 += ";";
                  _sendData1 += MvAvgS;
                  _sendData1 += ";";
                  _sendData1 += MvAvgT;
                  for(c = 0; c <=2; c++){
                    _sendData1 += ";";
                    _sendData1 += d;
                    _sendData1 += ";";
                    _sendData1 += Irms_tot[d];
                    d++;
                  }
                  _sendData1 += "\"}]";            
                  char dataBuffer1[2048];
                  _sendData1.toCharArray(dataBuffer1,2048);
                  wifiHandling();
                  MQTThandling(); //renew connection to MQTT broker
                  mqttClient.beginMessage(topicPublish, retain, Qos, dubplicat);                      // creates a new message to be published.
                  mqttClient.printf(dataBuffer1);                             
                  mqttClient.endMessage(); // publish
                  
                  #ifdef DEBUG
                  Serial.print("Published: ");Serial.println(dataBuffer1);
                  #endif
                  
                  blink_led1();
                  if(b >= 10){
                    d = 0;             
                  }
                }
              }
            }           
      
            // Auto calibration process
            if((is_calibrate == false) && (instruction_calibrate == 1)){
              digitalWrite(13,HIGH); // on led indicator for calibarion process
              float Irms0_0_c = emon0_0.calcIrms(1800); 
              float Irms0_1_c = emon0_1.calcIrms(1800);
            
              m_irms0[j] += y_irms[j]/Irms0_0_c;
              m_irms1[j] += y_irms[j+15]/Irms0_1_c; 
      
              #ifdef DEBUG
              Serial.print("M0:");Serial.print(m_irms0[j],5);Serial.print("|");Serial.print("M1:");Serial.println(m_irms1[j],5); 
              Serial.print(q);Serial.print("|");Serial.println(is_calibrate);
              Serial.print("|Irms0_c_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0_c,5);
              Serial.print("|Irms1_c_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1_c,5);
              #endif
  
              // Take a calibration factor avarage
              if((q == CAL_LITERATION)&&(i>=15)){
                is_calibrate = true;
                for(k = 0; k <= 15; k++){
                  m_irms0[k]=m_irms0[k]/(CAL_LITERATION+1);
                  m_irms1[k]=m_irms1[k]/(CAL_LITERATION+1);
  
                  #ifdef DEBUG
                  Serial.print("Avg_M");Serial.print(k);Serial.print(":");Serial.print(m_irms0[k],10);Serial.print("|");
                  Serial.print("Avg_M");Serial.print(k+16);Serial.print(":");Serial.println(m_irms1[k],10);
                  #endif
                } 
                if(k >= 15){
                  for(l = 0; l <= 32; l++){
                    if(m==2){ //loop 2x for divide
                      String _sendData2 = "[{\"dataCal\":\"";
                      _sendData2 += board_id;
                      _sendData2 += ";";
                      _sendData2 += instruction_calibrate;
                      for(n = 0; n <= 0; n++){                      
                        _sendData2 += ";";
                        _sendData2 += o; //channel
                        _sendData2 += ";";
                        _sendData2 += String(m_irms0[o],10); //Irms
                        o++;
                      }
                      for(n = 0; n <= 0; n++){                      
                        _sendData2 += ";";
                        _sendData2 += p+16; 
                        _sendData2 += ";";
                        _sendData2 += String(m_irms1[p],10);
                        p++;
                      }
                      _sendData2 += "\"}]";
                      char dataBuffer2[2048];
                      _sendData2.toCharArray(dataBuffer2, 2048);
                      wifiHandling();
                      MQTThandling(); //renew connection to MQTT broker
                      mqttClient.beginMessage(topicPublish2, retain, Qos, dubplicat);                      // creates a new message to be published.
                      mqttClient.printf(dataBuffer2);                             
                      mqttClient.endMessage(); //publish data
                      
                      #ifdef DEBUG
                      Serial.print("Published2: ");Serial.println(dataBuffer2);
                      #endif
                      
                      m = 0;
                    }
                    m++;
                  }
                } 
              }      
            } 
            i++;
            j=i;        
            if(i>15){
              i = 0;
              if(is_calibrate == false){
                j=i;
                q++;
              }
            }
          }
        } 
      }
    }
  }
}
