/*
 Basic MQTT example
 Date Create: 11/25/2021
 Create by: Parametrik
 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
  - publishes "value:0++" to the topic "ITSCourseOut"
  - subscribes to the topic "ITSCourseIn", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
*/

// #include <ESP8266WiFi.h>                                                           // Include library wifi client (ESP8266)
#include <WiFi.h>                                                                  // Include library wifi client (ESP32)
#include <PubSubClient.h>                                                          // Include library Publish and Subscribe MQTT by Nick O'Leary (Knolleary)

#define internalLED 2                                                              // Define internal LED pinout for ESP (ESP8266: GPIO2, ESP32: GPIO2)

// Define variable ssd, password, & mqtt_broker
const char* ssid = "RF_AP1";                                              
const char* password = "payungteduh123";
const char* mqtt_broker = "172.16.68.72";

WiFiClient espClient;                                                              // Instance name WiFiClient as espClient
PubSubClient client(espClient);                                                    // Instance name PubSubClient as client 

char msgOut[20];                                                                   // Define char_of_array buffer variable publish message
char msgIn[128];                                                                   // Define char_of_array buffer varibale subscribe message
long lastMsg = 0;                                                                  // Define long varibale for function milis() time calculation
int value = 0;                                                                     // Define varible value use as increamment calculation
String struv;                                                                      // Define varibale string use as convertion integer to string

void setup() {
  Serial.begin(9600);
  pinMode(internalLED, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);
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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    msgIn[(i - 0)] = (char)payload[i];
  }
  Serial.println();
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
      client.publish("ITSCourseOut", "start connected");
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
  if (!client.connected()) {
    reconnect();
  }

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;

    float Irms0_0 = 0.67;
    float Irms0_1 = 0.69;
    int i = 15;
    struv = "$" + String(i) + "_" +  String(Irms0_0) + ";" +
                  String(i+16) + "_" + String(Irms0_1) + ";";
//                  String(MvAvgR) + ";" +
//                  String(MvAvgS) + ";" +
//                  String(MvAvgT) + ";";
    struv.toCharArray(msgOut,20);
    
    Serial.print(" Message publish [");
    Serial.print("ITSCourseOut");
    Serial.print("]");
    Serial.println(msgOut);
    client.publish("ITSCourseOut", msgOut);

    client.subscribe("ITSCourseIn");
    Serial.print("Subs:");
    Serial.println(msgIn);
    value++;
  }

  String s = String(msgIn);
  if(s == "Led ON"){
    digitalWrite(internalLED, HIGH);
  } else {
    if(s == "Led OF"){
      digitalWrite(internalLED, LOW);
    }
  }
  client.loop();
}
