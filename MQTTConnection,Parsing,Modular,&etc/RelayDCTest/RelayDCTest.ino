#include <WiFi.h>

const int connection = 19; // Connection indicator (WiFi & MQTT)
const int processing = 18; // process calibration & measurement data voltage & current
const int Interval = 250;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(connection, OUTPUT);
  pinMode(processing, OUTPUT);
}

void loop() {
   for(int d=0; d<=1; d++){
    digitalWrite(connection, HIGH);Serial.println("connection ON!"); delay(500);
    digitalWrite(connection, LOW);Serial.println("connection OFF!"); delay(500);
    digitalWrite(processing, HIGH);Serial.println("Processing ON!"); delay(500);
    digitalWrite(processing, LOW);Serial.println("Processing OFF!"); delay(500);
  }
}
