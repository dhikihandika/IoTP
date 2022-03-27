/* 
    ESP Wifi client for ITSKedaireka Course
    Date Create: 11/25/2021
    Create by: Parametrik
*/


 #include <ESP8266WiFi.h>                                                           // Include library wifi client (ESP8266)
//#include <WiFi.h>                                                                  // Include library wifi client (ESP32)
#define LEDInternalESP 2

const char* ssid     = "RF_AP1";
const char* password = "payungteduh123";

void setup()
{
    Serial.begin(9600);
    pinMode(LEDInternalESP, OUTPUT);
    delay(10);
    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi Connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop(){
    if (WiFi.status() == WL_CONNECTED){
        digitalWrite(LEDInternalESP, HIGH);
        delay(1000);
        digitalWrite(LEDInternalESP, LOW);
        delay(500);
        Serial.println("Wifi Connected!");
    } else {
        digitalWrite(LEDInternalESP, LOW);
        Serial.println("Wifi Can't Connected!");
    }
}
