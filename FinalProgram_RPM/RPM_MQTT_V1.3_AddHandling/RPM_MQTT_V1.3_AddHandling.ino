#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoMqttClient.h>

//define board id
unsigned int idRPM = 35;    //Wheel 1

//WiFi Config
IPAddress local_IP(172, 16, 68, 202);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//MQTT Config
const char broker[] = "172.16.68.72";
int        port     = 1883;
const char topic[]  = "wheel1/rpm/data/subs1";

//Define digital pinout moude sensor and led indicator 
const int sensorPIN = 5;
const int statusPIN = 4;

int f,g,h = 0;
int rev = 0;
int sensorStateLast;
const int constant = 20;
unsigned long previousMillis = 0;        // will store last val = 0
const long interval = 10000;             // interval at which to reset is 10000 ms / 10 s

void setup() {
  Serial.begin(115200);
  pinMode(sensorPIN, INPUT);
  pinMode(statusPIN, OUTPUT);
  
  // Initialiaze connection to WiFi, MQTT and Schema MQTT Subscribtion Message
  wifiSetup();
  wifiHandling();
  MQTTconnection();
  OTA();
}

void OTA(){
   //Upload Program Over The Air (OTA)
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
}

void wifiSetup() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //WiFi init
  #ifdef DEBUG
  Serial.print("Connecting");
  Serial.println();
  #endif
  wifiHandling();
}

void wifiHandling() {
  while(WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print(".");
    blink_led2();
    if(g==60){ // +- 60 s
      Serial.println("CAN'T CONNECT TO AP, ESP8266 SOFTWARE RESET!");
      ESP.restart();
    }
    g++;
  }
  while(WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  if(WiFi.status() == WL_CONNECTED) { //WiFi connected
    #ifdef DEBUG
    Serial.println("");
    Serial.print("Wifi Connected, IP address: ");
    Serial.println(WiFi.localIP());
    #endif
  }
}

void MQTTconnection() {
  // Each client must have a unique client ID
  mqttClient.setId("ESP8266S_GMI_RPM_35");
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  MQTThandling();
}

void MQTThandling() { // handling renew connection MQTT Client "ESP32" to Broker
  if (!mqttClient.connect(broker, port)) {
    blink_led3();
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    if(mqttClient.connectError()){
      mqttClient.connect(broker, port);
    }
    f++;
    Serial.print("f:");Serial.println(f);
  }
}

void blink_led1() { // Indicator for publish data VC
  if(rev >= 0){
    digitalWrite(statusPIN, HIGH); delay(500);
    digitalWrite(statusPIN, LOW); delay(500);    
  }
}

void blink_led2() { // Indicator for lost/trouble in WiFi connection
  for(h=0; h<=1; h++){
    digitalWrite(statusPIN, HIGH); delay(500);
    digitalWrite(statusPIN, LOW); delay(500);
  }
}

void blink_led3() { // Indicator for los MQTT connection
  for(h=0; h<=3; h++){
    digitalWrite(statusPIN, HIGH); delay(250);
    digitalWrite(statusPIN, LOW); delay(250);
  }
}

void loop() {
  if (!mqttClient.connected()) {
    MQTThandling();
  } else {
    // call poll() regularly to allow the library to send MQTT keep alive which
    // avoids being disconnected by the broker
    mqttClient.poll();
      unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you reset rev
      previousMillis = currentMillis;
      Serial.print("Rev_Now:");
      Serial.println(rev);
      int RPM = (rev*6) + constant; // rps every interval: 10000 ms
      String _sendData = "{\"sensor_id\":\"";
      _sendData += idRPM;
      _sendData += "\",\"sensor_type\":\"1";
      _sendData += "\",\"rpm\":\"";
      _sendData += RPM;
      _sendData += "\"}";
      char dataBuffer[1024];
      _sendData.toCharArray(dataBuffer, 1024);
      wifiHandling();
      if(!mqttClient.connected()) { // handling MQTT connection before publish data
        MQTThandling();
      }
      mqttClient.beginMessage(topic);
      mqttClient.print(dataBuffer);
      mqttClient.endMessage();
      blink_led1();
      Serial.println("Data Send!\n");
      rev = 0;
    }
    if (digitalRead(sensorPIN) == 0){
      do{
        sensorStateLast = digitalRead(sensorPIN);
        delay(10);
      } while (sensorStateLast == 0);
      rev++;
      Serial.println(rev);
    }
    f = 0;
  }
  if(f==60){ // +- 60 s
    Serial.println("CAN'T CONNECT TO BROKER, ESP8266 SOFTWARE RESET!");
    ESP.restart();
  } 
}
