#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

//define board id
unsigned int idRPM = 35;    //Wheel 1
//String IP = "172.16.68.208";
int16_t NId1;
int16_t NId2;
int16_t HId1;
int16_t HId2;

const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

void setup() {
  Serial.begin(115200);
  
  //WiFi Config
  NId1 = 172;NId2 = 16;HId1 = 68;HId2 = 229;
  IPAddress local_ip(NId1, NId2, HId1, HId2);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress gateway(172, 16, 68, 1);
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);

  // Initialiaze connection to WiFi, MQTT and Schema MQTT Subscribtion Message
  if (!WiFi.config(local_ip, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //WiFi init
  Serial.print("Connecting");
  Serial.println();
    while(WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    Serial.print(".");
  }
  while(WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  if(WiFi.status() == WL_CONNECTED) { //WiFi connected
    Serial.println("");
    Serial.print("Wifi Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void loop() {
    String B;
    B += HId1;
    B += HId2;
    Serial.println(WiFi.localIP());
    Serial.println(B);
    delay(1000);
}
