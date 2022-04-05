#include "EmonLib.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <SimpleKalmanFilter.h>
#include <ArduinoOTA.h>

//#define DEBUG
#define CAL_LITERATION 48 //literation for calibration 48x16s=768s | 768s/60s=12,8m
#define AvgSize 200

// Define board id
unsigned int board_id; // this is Host id IP

// Static WiFi config (172.16.68.203)
int16_t NId1 = 172;
int16_t NId2 = 16;
int16_t HId1 = 68;
int16_t HId2 = 203;

// Define Static WiFi config
IPAddress local_IP(NId1, NId2, HId1, HId2);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 16, 68, 1);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// MQTT config
const char* ssid = "RF_AP1"; //AP SSID
const char* password = "payungteduh123"; //AP PASSWORD

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

const char broker[]         = "172.16.68.72";
int        port             = 1883;
const char topicPublish3[]  = "ponds/power_meter/startup/subs";
char topicSubscribe[2048];
String subCal = "ponds/power_meter/calibration/pubs/";
String _subCalTopics;
const char topicPublish2[]  = "ponds/power_meter/calibration/subs";
const char topicPublish[]   = "ponds/power_meter/data/subs";

unsigned long KeepAliveInterval = 60;
unsigned long ConnectionTimeOutInterval = 30;
bool retain = false;
int Qos = 1;
bool dubplicat = false;

// Define pinout indicator led & DC relay
const int boardInd = 13;
const int connection = 18; // Connection indicator (WiFi & MQTT)
const int processing = 19; // process calibration & measurement data voltage & current

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
char *mux[] = {"0000", //0   ch0
               "1000", //0   ch1
               "0100", //0   ch2
               "1100", //0   ch3
               "0010", //0   ch4
               "1010", //0   ch5
               "0110", //0   ch6
               "1110", //0   ch7
               "0001", //0   ch8
               "1001", //0   ch9
               "0101", //0   ch10
               "1101", //0   ch11
               "0011", //0   ch12
               "1011", //0   ch13
               "0111", //0   ch14
               "1111"  //0   ch15
              };

// Define MUX variable
String ch_mux;
String mux_bit0,mux_bit1,mux_bit2,mux_bit3;

// Buffer, parse and calibration_instruction variable
String dataIn = "";
String dt[33];
int instruction_calibrate = 33; //no instruction
bool is_calibrate;
bool close_doorBuffer;

int r, s, t, u, v = 0; //for parsing data
int a, b, c, d, e, f, g = 0; //for voltage Measure, indicator, wdt
int h, i, j, k, l, m, n, o, p, q = 0; // for auto calibration and send data Cal

// Voltage sensor variable
float peakRS, peakST, peakTR = 0;
float peakBeforeRS, peakBeforeST, peakBeforeTR = 0;
float vmaxRS, vmaxST, vmaxTR = 0;
// Moving avarage variable
uint8_t index0, index1, index2 = 0;
float sum0, sum1, sum2;
float reading0[AvgSize],reading1[AvgSize],reading2[AvgSize];
float vRS, vST, vTR = 0;
float MvAvgRS, MvAvgST, MvAvgTR = 0;
float V380RS, V380ST, V380TR = 0;
float Vrms380 = 394.00;     // Take form measurement tools
float Vrms220RS = 218.40;     // Take from measurement tools
float Vrms220ST = 217.00;     // Take from measurement tools
float Vrms220TR = 217.60;     // Take from measurement tools
float fc_v220RS = 1.961467654;
float fc_v220ST = 1.843550553;
float fc_v220TR = 1.832236936;

// Current sensor variable
float Irms0_0[16],Irms0_1[16],Irms_tot[33];
float y_irms[32];                                                             // buffer variable for constant calibartion (y)
float m_irms0[16],m_irms1[16];                                                // buffer variable for factor calibration mux0 & mux1 (m)
String fc_0; String fc_1;                                                     // buffer string variable for factor calibration mux1 (m)

// Buffer old millis time
int64_t oldtime;

// Emon Init
EnergyMonitor emon0_0;                            // Create an instance emon0
EnergyMonitor emon0_1;                            // Create an instance emon1

TaskHandle_t Task1;
TaskHandle_t Task2;

//======================================= INITIAILZE PROGRAM (SETUP) =================================================//
void setup() {
  Serial.begin(9600); 
  Serial2.begin(115200); 

  // Initialize board_id
  String B;
  B += HId1;
  B += HId2;
  board_id = B.toInt();

  // Initialize topic subscribe calibration (unique board_id)
  _subCalTopics = subCal;
  _subCalTopics += String(board_id);
  _subCalTopics.toCharArray(topicSubscribe,2048);

  // Initialiaze energy monitoring library
  emon0_0.current(A4, 38.3);                       // Current: input pin, calibration. ratio/Rburden
  emon0_1.current(A7, 38.3);                       // Current: input pin, calibration. ratio/Rburden
  
  // Initialiaze digital pinout for indicator
  pinMode(boardInd, OUTPUT);
  pinMode(connection, OUTPUT);
  pinMode(processing, OUTPUT);

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
  wifiHandling();
  MQTTconnection();
  mqttClient.poll();
  OTA();
  is_calibrate = false;
  close_doorBuffer = false;

  // Initialize value oldtime millis
  Serial.println("START IoT DEVICE !!!");
  Serial.print("board_id:");Serial.println(board_id);
  oldtime = millis();
  
  // Create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  // Create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}

//======================================= FUNCTION & PROCEDURE =================================================//
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
}

void wifiHandling() {
  while(WiFi.status() != WL_CONNECTED) { //WiFi not connected
    delay(500);
    blink_led2();
    wifiSetup();
    if(g==25){ // +- 60 s
      Serial.println("CAN'T CONNECT TO WIFI, ESP32 SOFTWARE RESET!");
      ESP.restart();
    } 
    g++;
    Serial.print("g:");Serial.println(g);
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
  mqttClient.setId("ESP32S_GMI_VC_" + board_id);
//  mqttClient.setKeepAliveInterval(KeepAliveInterval);
//  mqttClient.setConnectionTimeout(ConnectionTimeOutInterval);
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
   Serial.println("OTA Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
}

void subsMQTTMessage() {
  if (!mqttClient.connected()) {
    MQTThandling();
  } else {
    String _sendStart = "{\"board_id\":\"";
    _sendStart += board_id;
    _sendStart += "\"}";
    char dataBufferStart[2048];
    _sendStart.toCharArray(dataBufferStart, 2048);
    wifiHandling();
    if (!mqttClient.connected()) { // handling MQTT connection when autocalibratin proces run
      MQTThandling();
    }
    mqttClient.beginMessage(topicPublish3, retain, Qos, dubplicat);                      // creates a new message to be published.
    mqttClient.printf(dataBufferStart);
    mqttClient.endMessage(); // publish


    mqttClient.onMessage(onMqttMessage);   // Set the message receive callback
    #ifdef DEBUG
    Serial.print("Subscribing to topic: ");
    Serial.println(topicSubscribe);
    Serial.println();
    #endif

    mqttClient.subscribe(topicSubscribe, Qos); // subscribe to a topic

    #ifdef DEBUG
    Serial.print("Waiting for messages on topic: ");
    Serial.println(topicSubscribe);
    Serial.println();
    #endif
    f = 0;
  }
}

void onMqttMessage(int messageSize) {
  // use the Stream interface to print the contents
  if (mqttClient.available()) {
    dataIn = mqttClient.readString();
    parse_data();
    dataIn = "";
    close_doorBuffer = true;
  }

  #ifdef DEBUG
  Serial.println();
  #endif
}

void parse_data() {
  r = 0;

  #ifdef DEBUG
  Serial.print("in_data: ");
  Serial.println(dataIn);
  #endif

  // Initialize variabel, (variable reset)
  for (s = 0; s <= 31; s++) {
    dt[s] = "";
  }
  // Process parse data
  for (t = 1; t < dataIn.length(); t++) { // Character check every char data
    if ((dataIn[t] == '$') || (dataIn[t] == ';')) {  // Increment r, use for change index array buffer
      r++;
      dt[r] = "";     // Initialize array dt[j]
    } else {
      dt[r] = dt[r] + dataIn[t];     // Process tampung data
    }
  }

  // Save data parse to valiable dloat array 'y_irms[u]'
  instruction_calibrate = dt[0].toInt();

  #ifdef DEBUG
  Serial.print("status:"); Serial.println(dt[0].toInt());
  #endif

  if (instruction_calibrate == 0) { // NO instruction to calibrate
    for (u = 0; u <= 15; u++) {
      m_irms0[u] = dt[u + 1].toFloat();
      m_irms1[u] = dt[u + 17].toFloat();

      #ifdef DEBUG
      Serial.print(m_irms0[u], 10); Serial.print("|"); Serial.println(m_irms1[u], 10);
      #endif
    }
  } else {
    if (instruction_calibrate == 1) { // YES instruction to calibrate
      for (u = 0; u <= 31; u++) {
        y_irms[u] = dt[u + 1].toFloat();

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

void voltageMeasure() {
  //MeasureRS
  float real_valueRS = analogRead(A0);
  float measured_valueRS = real_valueRS + random(-100, 100) / 100.0;
  float peakRS = simpleKalmanFilter.updateEstimate(measured_valueRS);
  if (peakRS > peakBeforeRS) {// Measure voltage RS
    peakBeforeRS = peakRS;
    vmaxRS = peakRS;
  }
  if (peakRS < peakBeforeRS) {
    peakBeforeRS = peakRS;
    vRS = (0.2395 * vmaxRS) - 760.37 ; //Liner Regression y = 0,2395x - 760,37
    if (vRS > 0) {
      sum0 = sum0 - reading0[index0];
      reading0[index0] = vRS;
      sum0 = sum0 + reading0[index0];
      index0 = index0 + 1;
      if (index0 >= AvgSize) {
        index0 = 0;
      }
//      MvAvgRS = sum0 / AvgSize;
      MvAvgRS = (sum0 / AvgSize) * fc_v220RS;
      V380RS = Vrms380 * MvAvgRS / Vrms220RS;
    }
  }

  float real_valueST = analogRead(A3);
  float measured_valueST = real_valueST + random(-100, 100) / 100.0;
  float peakST = simpleKalmanFilter.updateEstimate(measured_valueST);
  //MeasureST
  if (peakST > peakBeforeST) {// Measure voltage ST
    peakBeforeST = peakST;
    vmaxST = peakST;
  }
  if (peakST < peakBeforeST) {
    peakBeforeST = peakST;
    vST = (0.2395 * vmaxST) - 760.37; // Liner Regression
    if (vST > 0) {
      sum1 = sum1 - reading1[index1];
      reading1[index1] = vST;
      sum1 = sum1 + reading1[index1];
      index1 = index1 + 1;
      if (index1 >= AvgSize) {
        index1 = 0;
      }
//      MvAvgST = sum1 / AvgSize;
      MvAvgST = (sum1 / AvgSize) * fc_v220ST;
      V380ST = Vrms380 * MvAvgST / Vrms220ST;
    }
  }

  float real_valueTR = analogRead(A6);
  float measured_valueTR = real_valueTR + random(-100, 100) / 100.0;
  float peakTR = simpleKalmanFilter.updateEstimate(measured_valueTR);
  //MeasureTR
  if (peakTR > peakBeforeTR) {// Measure voltage TR
    peakBeforeTR = peakTR;
    vmaxTR = peakTR;
  }
  if (peakTR < peakBeforeTR) {
    peakBeforeTR = peakTR;
    vTR = (0.2395 * vmaxTR) - 760.37; // Liner Regression
    if (vTR > 0) {
      sum2 = sum2 - reading2[index2];
      reading2[index2] = vTR;
      sum2 = sum2 + reading2[index2];
      index2 = index2 + 1;
      if(index2 >= AvgSize){
         index2 = 0;
      }
//      MvAvgTR = sum2 / AvgSize;      
      MvAvgTR = (sum2 / AvgSize) * fc_v220TR;
      V380TR = Vrms380 * MvAvgTR / Vrms220TR;
    }
  }
}

void blink_led1() { // Indicator for publish data VC
  digitalWrite(processing, HIGH);digitalWrite(boardInd, HIGH); delay(500);
  digitalWrite(processing, LOW);digitalWrite(boardInd, LOW); delay(500);
}

void blink_led2() { // Indicator for lost/trouble in WiFi connection
  for(d=0; d<=1; d++){
    digitalWrite(connection, HIGH);digitalWrite(boardInd, HIGH); delay(500);
    digitalWrite(connection, LOW);digitalWrite(boardInd, LOW); delay(500);
  }
}

void blink_led3() { // Indicator for los MQTT connection
  for(d=0; d<=3; d++){
    digitalWrite(connection, HIGH);digitalWrite(boardInd, HIGH); delay(250);
    digitalWrite(connection, LOW);digitalWrite(boardInd, LOW); delay(250);
  }
}

//========================================== MULTITHREAD  =================================================//
//Task1code: for only measure Voltage Sensor
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    voltageMeasure();
    Serial2.print(MvAvgRS); Serial2.print(","); Serial2.print(MvAvgST); Serial2.print(","); Serial2.println(MvAvgTR);
//    Serial.print(V380RS); Serial.print(","); Serial.print(V380ST); Serial.print(","); Serial.println(V380TR);
//    Serial.print(vRS);Serial.print(",");Serial.print(vST);Serial.print(",");Serial.println(vTR);
    delay(1);
  } 
}

//Task2code: Measure Current and send data to server
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    // Call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();
  
    if (close_doorBuffer == false) { // Check would doorBuffer is open
      subsMQTTMessage();
    } else {
      if (close_doorBuffer == true) {
        mqttClient.unsubscribe(topicSubscribe);   // unsubscribed to a topic
        if (!mqttClient.connected()) { // handling MQTT connection when autocalibratin proces run
          MQTThandling();
        } else {
          digitalWrite(connection, HIGH); // connected to WiFi and MQTT Broker
          if ((millis() - oldtime) > 0) {
            if ((millis() - oldtime) >= 1000) {
              oldtime = millis();
              ch_mux = mux[i];
              mux_bit0 = ch_mux.substring(0, 1); mux_bit1 = ch_mux.substring(1, 2); mux_bit2 = ch_mux.substring(2, 3); mux_bit3 = ch_mux.substring(3, 4);
  
              #ifdef DEBUG
//              Serial.println(ch_mux);
              Serial.print("MuxSig:");Serial.print(mux_bit0); Serial.print("|"); Serial.print(mux_bit1); Serial.print("|"); Serial.print(mux_bit2); Serial.print("|"); Serial.println(mux_bit3);
              #endif
  
              digitalWrite(S0_0, mux_bit0.toInt()); digitalWrite(S1_0, mux_bit1.toInt()); digitalWrite(S2_0, mux_bit2.toInt()); digitalWrite(S3_0, mux_bit3.toInt());
              digitalWrite(S0_1, mux_bit0.toInt()); digitalWrite(S1_1, mux_bit1.toInt()); digitalWrite(S2_1, mux_bit2.toInt()); digitalWrite(S3_1, mux_bit3.toInt());
  
              // Scema read data and send every 15 seconds
              if (((is_calibrate == true) && (instruction_calibrate == 1)) || ((is_calibrate == false) && (instruction_calibrate == 0))) {
                digitalWrite(processing, LOW);digitalWrite(boardInd, LOW);// on led indicator for calibarion process
                fc_0 = m_irms0[i]; fc_1 = m_irms1[i];
                Irms0_0[i] = emon0_0.calcIrms(1800) * fc_0.toFloat();
                Irms0_1[i] = emon0_1.calcIrms(1800) * fc_1.toFloat();
  
                #ifdef DEBUG
                Serial.print(m_irms0[i]); Serial.print("|"); Serial.print(m_irms1[i]);
                Serial.print("|Irms0_"); Serial.print(i); Serial.print("="); Serial.print(Irms0_0[i], 5);
                Serial.print("|Irms1_"); Serial.print(i + 16); Serial.print("="); Serial.println(Irms0_1[i], 5);
                #endif
  
                if (i >= 15) {
                  String _sendData1 = "[{\"dataVC\":\"";
                  _sendData1 += board_id;
                  _sendData1 += ";";
                  _sendData1 += V380RS;
                  _sendData1 += ";";
                  _sendData1 += V380ST;
                  _sendData1 += ";";
                  _sendData1 += V380TR;
                  for (h = 0; h <= 15; h++) {
                    _sendData1 += ";";
                    _sendData1 += Irms0_0[h];
                  }
                  for (h = 0; h <= 15; h++) {
                    _sendData1 += ";";
                    _sendData1 += Irms0_1[h];
                  }
                  _sendData1 += ";\"}]";
                  char dataBuffer1[2048];
                  _sendData1.toCharArray(dataBuffer1, 2048);
                  wifiHandling();
                  if (!mqttClient.connected()) { // handling MQTT connection when autocalibratin proces run
                    MQTThandling();
                  }
                  mqttClient.beginMessage(topicPublish, _sendData1.length(), retain, Qos, dubplicat);                      // creates a new message to be published.
                  mqttClient.printf(dataBuffer1);
                  mqttClient.endMessage(); // publish
  
                  #ifdef DEBUG
                  Serial.print("Published: "); Serial.println(dataBuffer1);
                  #endif
  
                  blink_led1();
                }
              }
  
              // Auto calibration process
              if ((is_calibrate == false) && (instruction_calibrate == 1)) {
                digitalWrite(processing, HIGH);digitalWrite(boardInd, HIGH);// on led indicator for calibarion process
                float Irms0_0_c = emon0_0.calcIrms(1800);
                float Irms0_1_c = emon0_1.calcIrms(1800);
  
                m_irms0[j] += y_irms[j] / Irms0_0_c;
                m_irms1[j] += y_irms[j + 15] / Irms0_1_c;
  
                #ifdef DEBUG
                Serial.print("M0:"); Serial.print(m_irms0[j], 5); Serial.print("|"); Serial.print("M1:"); Serial.println(m_irms1[j], 5);
                Serial.print("Litertion|IsCalibrate:");Serial.print(q); Serial.print("|"); Serial.println(is_calibrate);
                Serial.print("|Irms0_c_"); Serial.print(i); Serial.print("="); Serial.print(Irms0_0_c, 5);
                Serial.print("|Irms1_c_"); Serial.print(i + 16); Serial.print("="); Serial.println(Irms0_1_c, 5);
                #endif

                //Handling 
                // Take a calibration factor avarage
                if ((q == CAL_LITERATION) && (i >= 15)) {
                  is_calibrate = true;
                  for (k = 0; k <= 15; k++) {
                    m_irms0[k] = m_irms0[k] / (CAL_LITERATION + 1);
                    m_irms1[k] = m_irms1[k] / (CAL_LITERATION + 1);
  
                    #ifdef DEBUG
                    Serial.print("Avg_M"); Serial.print(k); Serial.print(":"); Serial.print(m_irms0[k], 10); Serial.print("|");
                    Serial.print("Avg_M"); Serial.print(k + 16); Serial.print(":"); Serial.println(m_irms1[k], 10);
                    #endif
                  }
                  if (k >= 15) {
                    for (l = 0; l <= 32; l++) {
                      if (m == 2) { //loop 2x for divide
                        String _sendData2 = "[{\"dataCal\":\"";
                        _sendData2 += board_id;
                        _sendData2 += ";";
                        _sendData2 += 0;
                        for (n = 0; n <= 0; n++) {
                          _sendData2 += ";";
                          _sendData2 += o; //channel
                          _sendData2 += ";";
                          _sendData2 += String(m_irms0[o], 10); //Irms
                          o++;
                        }
                        for (n = 0; n <= 0; n++) {
                          _sendData2 += ";";
                          _sendData2 += p + 16;
                          _sendData2 += ";";
                          _sendData2 += String(m_irms1[p], 10);
                          p++;
                        }
                        _sendData2 += ";\"}]";
                        char dataBuffer2[2048];
                        _sendData2.toCharArray(dataBuffer2, 2048);
                        wifiHandling();
                        if (!mqttClient.connected()) { // handling MQTT connection when autocalibratin proces run
                          MQTThandling();
                        }
                        mqttClient.beginMessage(topicPublish2, _sendData2.length(), retain, Qos, dubplicat);                      // creates a new message to be published.
                        mqttClient.printf(dataBuffer2);
                        mqttClient.endMessage(); //publish data
  
                        #ifdef DEBUG
                        Serial.print("Published2: "); Serial.println(dataBuffer2);
                        #endif
                        delay(200);
                        m = 0;
                      }
                      m++;
                    }
                  }
                }
              }
              i++;
              j = i;
              if (i > 15) {
                i = 0;
                if (is_calibrate == false) {
                  j = i;
                  q++;
                }
              }
            }
          } else { // when (millis - oldtime < 0)
            if(((4320000000 - oldtime) + millis())  >= 1000){ //((approx - oldtime) + millis())
              oldtime = millis();
            }
          }
          f = 0;
        }
      }
    }
    if(f==60){ // +- 60 s
      Serial.println("CAN'T CONNECT TO BROKER, ESP32 SOFTWARE RESET!");
      ESP.restart();
    } 
    delay(1);
  }
}

void loop() {
}
