/*
  ArduinoMqttClient - WiFi Simple Receive Callback

  This example connects to a MQTT broker and subscribes to a single topic.
  When a message is received it prints the message to the serial monitor,
  it uses the callback functionality of the library.

  The circuit:
  - Arduino MKR 1000, MKR 1010 or Uno WiFi Rev.2 board

  This example code is in the public domain.
*/

#include <ArduinoMqttClient.h>
#include <WiFi.h>

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
const char topic[]  = "wheel1/power_meter/esp32/received";
const char topic2[] = "real_unique_topic_2";
const char topic3[] = "real_unique_topic_3";

String dataIn = " ";
String dt[33];
float y_irms[32];
float Irms0[16]; // buffer variable for factor calibration mux0 (m)
float Irms1[16]; // buffer variable for factor calibration mux1 (m)
int i;
int instruction_calibrate;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialiaze connection to WiFi and MQTT
  wifiSetup();
  MQTTonnection();

  // SUBSCRIBE DATA
  // Set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  // Subscribe to a topic
  mqttClient.subscribe(topic);

  // Topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();
}

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

void MQTTonnection(){
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
//  Serial.println("Received a message with topic:'");
//  Serial.print(mqttClient.messageTopic());
//  Serial.print("'|Length: ");
//  Serial.print(messageSize);
//  Serial.println(" Bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
//    Serial.print((char)mqttClient.read());
    dataIn = mqttClient.readString();
    parse_data();
    dataIn="";
  }
  Serial.println();
}

void parse_data(){
  int j=0;
  
  Serial.print("in_data: ");
  Serial.println(dataIn);
   
  // Initialize variabel, (variable reset)
  for (int l = 0; l <= 32; l++){
    dt[l]="";
  }
  // Process parse data
  for(i=1; i<dataIn.length(); i++){
    // Character check every char data
    if((dataIn[i] == '$') || (dataIn[i] == ';')){
    // Increment j, use for change index array buffer
    j++;
    dt[j]="";       // Initialize array dt[j]
    }else{
    // Process tampung data
    dt[j] = dt[j] + dataIn[i];
    }
  }
  // Save data parse to valiable dloat array 'y_irms[k]'
  instruction_calibrate = dt[0].toInt();
  Serial.print("status:");Serial.println(instruction_calibrate); 
  if(instruction_calibrate == 0){
    for (int k = 0; k <= 15; k++){
      Irms0[k+1] = dt[k+1].toFloat();
      Irms1[k+1] = dt[k+17].toFloat();
      Serial.print(Irms0[k+1]); Serial.print("|"); Serial.println(Irms1[k+1]);
    }
  } else {
    if(instruction_calibrate == 1){
      for (int k = 0; k <= 31; k++){
        y_irms[k+1] = dt[k+1].toFloat();
        Serial.println(y_irms[k+1]);
      }
    }
  }
}

void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();
}
