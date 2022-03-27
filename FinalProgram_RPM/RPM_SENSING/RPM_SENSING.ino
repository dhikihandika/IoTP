#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
//#include <ArduinoOTA.h>
#include <WiFiUdp.h>

const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD
const char* UDP_SERVER = "172.16.68.72"; //UDP server IP
int UDP_PORT = 9999; //UDP server port
WiFiUDP UDP; //UDP init

//WiFi config
IPAddress local_IP(172, 16, 68, 200);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

int idRPM = 1;    //Wheel 1
int rev = 0;
int sensorStateLast;
int sensorPIN = 5;
int statusPIN = 4;
int sensor_id = 2;
bool data = true;
unsigned long endTime;
unsigned long elapsedTime;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last val = 0


// constants won't change:
const long interval = 10000;           // interval at which to reset is 10000 ms / 10 s
//const int betweenReadings = 500;       // interval at which it will read your variable

void setup() {
  Serial.begin(115200);
  pinMode(sensorPIN, INPUT);
  pinMode(statusPIN, OUTPUT);
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password); //WiFi init
  Serial.print("Connecting");
  Serial.println();
  while (WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print("=");
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) { //WiFi connected
    Serial.println("Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }

//   //Upload Program Over The Air (OTA)
//   ArduinoOTA.onStart([]() {
//     String type;
//     if (ArduinoOTA.getCommand() == U_FLASH)
//       type = "sketch";
//     else // U_SPIFFS
//       type = "filesystem";

//     // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//     Serial.println("Start updating " + type);
//   });
//   ArduinoOTA.onEnd([]() {
//     Serial.println("\nEnd");
//   });
//   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//   });
//   ArduinoOTA.onError([](ota_error_t error) {
//     Serial.printf("Error[%u]: ", error);
//     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//     else if (error == OTA_END_ERROR) Serial.println("End Failed");
//   });
//   ArduinoOTA.begin();
//   Serial.println("Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
}

void status_UPDATE() {
  if (rev >= 0) {
    digitalWrite(statusPIN, HIGH);
    delay(200);
    digitalWrite(statusPIN, LOW);
    delay(100);
    digitalWrite(statusPIN, HIGH);
    delay(200);
    digitalWrite(statusPIN, LOW);
    //Serial.println("STATUS 0");
  }
}

void loop() {
  // ArduinoOTA.handle();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you reset rev
    previousMillis = currentMillis;
    Serial.print("Rev_Now:");
    Serial.println(rev);
    int RPM = rev*6;
    String _sendData = "{\"sensor_id\":\"";
    _sendData += idRPM;
    _sendData += "\",\"sensor_type\":\"1";
    _sendData += "\",\"rpm\":\"";
    _sendData += RPM;
    _sendData += "\"}";
    char dataBuffer[1024];
    _sendData.toCharArray(dataBuffer, 1024);
    UDP.beginPacket(UDP_SERVER, UDP_PORT);
    UDP.printf(dataBuffer);
    UDP.endPacket();
    status_UPDATE();
    Serial.println("Data Send!\n");
    rev = 0;
  }
  if (digitalRead(sensorPIN) == 0) {
    do{
      sensorStateLast = digitalRead(sensorPIN);
      delay(10);
    }
    while (sensorStateLast == 0);
    rev++;
    Serial.println(rev);
  }
}
