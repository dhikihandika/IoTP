#include "EmonLib.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>

#define DEBUG

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
const char willTopic[] = "wheel1/power_meter/sensor/will";
const char topicPublish[]  = "wheel1/power_meter/sensor/subs1";
const char topicPublish2[]  = "wheel1/power_meter/calibration/subs2";
const char topicSubscribe[] = "wheel1/power_meter/calibration/pubs1";

// Buffer old millis time
int64_t oldtime;
int i = 0;

int sensor_idv = 1;
float MvAvgR = 221;
float MvAvgS = 222;
float MvAvgT = 223;
float Irms0_0[16] = {0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.10,0.11,0.12,0.13,0.14,0.15};
float Irms0_1[16] = {0.16,0.17,0.18,0.19,0.20,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.30,0.31};


//======================================= INITIAILZE PROGRAM (SETUP) =================================================//
void setup() {
  Serial.begin(9600);
  
  // Initialiaze connection to WiFi, MQTT and Schema MQTT Subscribtion Message 
  wifiSetup();
  MQTTconnection();
  
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

void MQTTconnection(){

  // You can provide a unique client ID, if not set the library uses Arduin-millis()
  // Each client must have a unique client ID
   mqttClient.setId("ESP32");

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  // By default the library connects with the "clean session" flag set,
  // you can disable this behaviour by using
  // mqttClient.setCleanSession(false);

  // set a will message, used by the broker when the connection dies unexpectantly
  // you must know the size of the message before hand, and it must be set before connecting
//  String willPayload = "oh no!";
//  bool willRetain = true;
//  int willQos = 0;
//
//  mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
//  mqttClient.print(willPayload);
//  mqttClient.endWill();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

//========================================== MAIN LOOP  =================================================//
void loop() {
  // Call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();
  if((millis() - oldtime) > 0){
     if((millis() - oldtime) >= 1000){
        oldtime = millis();
        Serial.print("Count:");Serial.println(i);
        
        if(i>=15){
          String _sendData1 = "[{\"sensor_id\":\"";
          _sendData1 += sensor_idv;
          _sendData1 += "\",\"voltage_r\":\"";
          _sendData1 += MvAvgR;
          _sendData1 += "\",\"voltage_s\":\"";
          _sendData1 += MvAvgS;
          _sendData1 += "\",\"voltage_t\":\"";
          _sendData1 += MvAvgT;
          for(int v = 0; v <= 15; v++){
             _sendData1 += "\",\"Irms_ch";
             _sendData1 += v;
             _sendData1 += "\":\"";
             _sendData1 += Irms0_0[v];
          }
          for (int w = 0; w <= 15; w++){
            _sendData1 += "\",\"Irms_ch";
            _sendData1 += w+16;
            _sendData1 += "\":\"";
            _sendData1 += Irms0_1[w];
          }
            _sendData1 += "\"}]";            
            char dataBuffer1[4096];
            _sendData1.toCharArray(dataBuffer1, 4096);
            mqttClient.beginMessage(topicPublish);                      // creates a new message to be published.
            mqttClient.printf(dataBuffer1);                             
            mqttClient.endMessage(); // publish
            Serial.print("Published: ");Serial.println(dataBuffer1);
            while (WiFi.status() != WL_CONNECTED) { //WiFi not connected
              delay(500);
              Serial.print(".");
              Serial.println("Wifi dead!");
            }
            if(WiFi.status() == WL_CONNECTED) { //WiFi connected
              Serial.println("Wifi alive!");
            }
//            while(!mqttClient.connect(broker, port)) {
//              Serial.print("MQTT connection failed! Error code = ");
//              Serial.println(mqttClient.connectError());
//              if(mqttClient.connectError() == -2){
//                Serial.println("Reconnectinng to broker");
//                delay(500);
//                Serial.print(".");
//                mqttClient.connect(broker, port);
//              }
//            } 
            if(!mqttClient.connect(broker, port)){
              Serial.print("MQTT connection code = ");
              Serial.println(mqttClient.connectError());
//              if(mqttClient.connectError() == 0){
//                Serial.println("MQTT broker alive!");
//              } else {
//                Serial.println("MQTT broker dead!");
//              }
            }
//            mqttClient.poll();
        }  
        i++;              
     }
     // Auto calibration process  
     if(i>15){
       i = 0;
     }
   }
 } 
