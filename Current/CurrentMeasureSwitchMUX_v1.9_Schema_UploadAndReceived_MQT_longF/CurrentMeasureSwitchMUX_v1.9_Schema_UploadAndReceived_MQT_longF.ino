#include "EmonLib.h"
#include <WiFi.h>
#include <PubSubClient.h>

#define DEBUG

//WiFi config
IPAddress local_IP(172, 16, 68, 201); //setting static ip
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// MQTT config
const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

WiFiClient espClient;
PubSubClient client(espClient);

const char broker[] = "172.16.68.72";
int        port     = 1883;
const char topicPublish[]  = "wheel1/power_meter/sensor/subs1";
const char topicPublish2[]  = "wheel1/power_meter/calibration/subs2";
const char topicSubscribe[] = "wheel1/power_meter/calibration/pubs1";

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

int cal_literation = 1;                                                      // literation for calibration
int p = 0;int i = 0;int j = 0;int q = 0; int t = 33; int u = 0;

int sensor_idv = 1;
float MvAvgR = 221;
float MvAvgS = 221;
float MvAvgT = 221;

float Irms0_0[16];
float Irms0_1[16];
float y_irms[32];                                                             // buffer variable for constant calibartion (y)
float m_irms0[16];                                                              // buffer variable for factor calibration mux0 (m)
float m_irms1[16];                                                              // buffer variable for factor calibration mux1 (m)
               
String fc_0;String fc_1;

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
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  is_calibrate = false; 
  close_doorBuffer = false;  
//  avb_dataInBuffer = false;
  
  // Initialize value oldtime millis
  Serial.println("START DEVICE");
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
  while (WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print(".");
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
}

//void MQTTconnection(){
//  Serial.print("Attempting to connect to the MQTT broker: ");
//  Serial.println(broker);
//  if (!mqttClient.connect(broker, port)) {
//    Serial.print("MQTT connection failed! Error code = ");
//    Serial.println(mqttClient.connectError());
//    while (1);
//  }
//}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topicPublish, "hallo saya masuk!");
      // ... and resubscribe
//      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

char dataBuf[128];
int y = 0;

void callback(char* topicSubscribe, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topicSubscribe);
  Serial.print("]:");
  for (int x = 0; x < length; x++) {
    Serial.print((char)payload[x]);
    dataBuf[] += (char)payload[x];
    y++;
  }
  if(y >= length){
    dataIn = String(dataBuf);
    parse_data();
    dataIn="";  
    close_doorBuffer = true;
  }
  Serial.println();
}

//void subsMQTTMessage(){
//    // Subscribe data schema
//  //  Serial.println("Delay ");
//  //  delay(5000);
//  mqttClient.onMessage(onMqttMessage);   // Set the message receive callback
//
//  Serial.print("Subscribing to topic: ");
//  Serial.println(topicSubscribe);
//  Serial.println();
//
//  mqttClient.subscribe(topicSubscribe); // subscribe to a topic 
//
//  Serial.print("Waiting for messages on topic: ");
//  Serial.println(topicSubscribe);
//  Serial.println();
//}

//void onMqttMessage(int messageSize) {
//  // use the Stream interface to print the contents
//  if(mqttClient.available()) {
////    Serial.print((char)mqttClient.read());
//    dataIn = mqttClient.readString();
//    parse_data();
//    dataIn="";  
//    close_doorBuffer = true;
//  }
//  Serial.println();
//}

void parse_data(){
  int r=0;
  
  Serial.print("in_data: ");
  Serial.println(dataIn);
   
  // Initialize variabel, (variable reset)
  for (int s = 0; s <= 31; s++){
    dt[s]="";
  }
  // Process parse data
  for(q = 1; q<dataIn.length(); q++){     // Character check every char data
    if((dataIn[q] == '$') || (dataIn[q] == ';')){    // Increment r, use for change index array buffer
    r++;
    dt[r]="";       // Initialize array dt[j]
    }else{
    dt[r] = dt[r] + dataIn[q];     // Process tampung data
    }
  }
  
  // Save data parse to valiable dloat array 'y_irms[t]'
  instruction_calibrate = dt[0].toInt();
  Serial.print("status:");Serial.println(dt[0].toInt());
  if(instruction_calibrate == 0){ // NO instruction to calibrate
    for (int m = 0; m <= 15; m++){
      m_irms0[m] = dt[m+1].toFloat();
      m_irms1[m] = dt[m+17].toFloat();
      Serial.print(m_irms0[m]); Serial.print("|"); Serial.println(m_irms1[m]);
    }
  } else {
    if(instruction_calibrate == 1){ // YES instruction to calibrate
      for (int n = 0; n <= 31; n++){
        y_irms[n] = dt[n+1].toFloat();
        Serial.println(y_irms[n]);
      }
      Serial.println("start calibration!");
    }
  }
}

void blink_led1(){
    digitalWrite(13, HIGH);delay(200);
    digitalWrite(13, LOW);delay(200);
}
void blink_led2(){
  for (int o = 0; o <= 1; o++){
    digitalWrite(13, HIGH);delay(400);
    digitalWrite(13, LOW);delay(400);
  }
}
void blink_led3(){
  for (int o = 0; o <= 1; o++){
    digitalWrite(13, HIGH);delay(2000);
    digitalWrite(13, LOW);
  }
}

//========================================== MAIN LOOP  =================================================//
void loop() {
  // Call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Check would doorBuffer is open
  if(close_doorBuffer == false){
//    client.setCallback(callback);
  } else {
    if(close_doorBuffer == true){
     client.unsubscribe(topicSubscribe);   // unsubscribed to a topic 
      
      if((millis() - oldtime) > 0){
        if((millis() - oldtime) >= 1000){
          oldtime = millis();
          ch_mux = mux[i];
          mux_bit0 = ch_mux.substring(0,1); mux_bit1 = ch_mux.substring(1,2); mux_bit2 = ch_mux.substring(2,3); mux_bit3 = ch_mux.substring(3,4);
    //      Serial.println(ch_mux);
    //      Serial.print(mux_bit0);Serial.print("|");Serial.print(mux_bit1);Serial.print("|");Serial.print(mux_bit2);Serial.print("|");Serial.println(mux_bit3);
              
          digitalWrite(S0_0,mux_bit0.toInt());digitalWrite(S1_0,mux_bit1.toInt());digitalWrite(S2_0,mux_bit2.toInt());digitalWrite(S3_0,mux_bit3.toInt());
          digitalWrite(S0_1,mux_bit0.toInt());digitalWrite(S1_1,mux_bit1.toInt());digitalWrite(S2_1,mux_bit2.toInt());digitalWrite(S3_1,mux_bit3.toInt()); 

          // scema read data and send every 15 seconds
          if(((is_calibrate == true) && (instruction_calibrate == 1)) || ((is_calibrate == false) && (instruction_calibrate == 0))){
            fc_0 = m_irms0[i];fc_1 = m_irms1[i];
            Irms0_0[i] = emon0_0.calcIrms(1800) * fc_0.toFloat(); 
            Irms0_1[i] = emon0_1.calcIrms(1800) * fc_1.toFloat(); 
            mqttClient.poll();

            #ifdef DEBUG
            Serial.print(m_irms0[i]);Serial.print("|");Serial.print(m_irms1[i]);
            Serial.print("|Irms0_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0,5);
            Serial.print("|Irms1_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1,5);
            #endif

            if(i>=15){
              String _sendData1 = "[{\"sensor_id\":\"";
              _sendData1 += sensor_idv;
              _sendData1 += "\",\"voltage_r\":\"";
              _sendData1 += MvAvgR;
              _sendData1 += "\",\"voltage_s\":\"";
              _sendData1 += MvAvgS;
              _sendData1 += "\",\"voltage_t\":\"";
              _sendData1 += MvAvgT;

              for(int v = 0; v
              _sendData1 += "\",\"Irms_ch";
              _sendData1 += i;
              _sendData1 += "\":\"";
              _sendData1 += Irms0_0[i];
              _sendData1 += "\",\"Irms_ch";
              _sendData1 += i+16;
              _sendData1 += "\":\"";
              _sendData1 += Irms0_1[i];
              _sendData1 += "\"}]";            
      //       Serial.println(_sendData1);
               char dataBuffer1[128];
               _sendData1.toCharArray(dataBuffer1, 128);
               Serial.print("Published[");;Serial.print(dataBuffer1);Serial.print("]:")
               client.publish(topicPublish, dataBuffer1);
               blink_led1();
             }          
           }           
    
          // Auto calibration process
          if((is_calibrate == false) && (instruction_calibrate == 1)){
            float Irms0_0_c = emon0_0.calcIrms(1800); 
            float Irms0_1_c = emon0_1.calcIrms(1800);
          
            m_irms0[j] += y_irms[j]/Irms0_0_c;
            m_irms1[j] += y_irms[j+15]/Irms0_1_c; 
    
            #ifdef DEBUG
            Serial.print("M0:");Serial.print(m_irms0[j],5);Serial.print("|");Serial.print("M1:");Serial.println(m_irms1[j],5); 
            Serial.print(p);Serial.print("|");Serial.println(is_calibrate);
            Serial.print("|Irms0_c_");Serial.print(i);Serial.print("=");Serial.print(Irms0_0_c,5);
            Serial.print("|Irms1_c_");Serial.print(i+16);Serial.print("=");Serial.println(Irms0_1_c,5);
            #endif
              
            if((p==cal_literation)&&(i>=15)){
              is_calibrate = true;
              for (int k = 0; k <= 15; k++){
                m_irms0[k]=m_irms0[k]/(cal_literation+1);
                m_irms1[k]=m_irms1[k]/(cal_literation+1);
    
                String _sendData2 = "[{\"sensor_id\":\"";
                _sendData2 += sensor_idv;
                _sendData2 += "\",\"status\":\"";
                _sendData2 += instruction_calibrate;
                _sendData2 += "\"},{\"ch\":\"";
                _sendData2 += k;
                _sendData2 += "\",\"m_irms0\":\"";
                _sendData2 += m_irms0[k];
            
                _sendData2 += "\"},{\"ch\":\"";
                _sendData2 += k+16;
                _sendData2 += "\",\"m_irms1\":\"";
                _sendData2 += m_irms1[k];
                _sendData2 += "\"}]";
            
      //        Serial.println(_sendData1);
                char dataBuffer2[2048];
                _sendData2.toCharArray(dataBuffer2, 2048);
                mqttClient.beginMessage(topicPublish2);                      // creates a new message to be published.
                mqttClient.printf(dataBuffer2);                             
                mqttClient.endMessage();
                mqttClient.poll();
                #ifdef DEBUG
                Serial.print("Published2: ");Serial.println(dataBuffer2);
                #endif
                blink_led3();

                #ifdef DEBUG
                Serial.print("Avg_M");Serial.print(k);Serial.print(":");Serial.print(m_irms0[k],10);Serial.print("|");
                Serial.print("Avg_M");Serial.print(k+16);Serial.print(":");Serial.println(m_irms1[k],10);
                Serial.println();
                #endif
              } 
              blink_led2();
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
  }
}
