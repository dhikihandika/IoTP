const int connection = 17; // Connection indicator (WiFi & MQTT)
const int processing = 16; // process calibration & measurement data voltage & current
const int Interval = 250;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(connection, OUTPUT);
  pinMode(processing, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(connection, HIGH);Serial.println("Connection ON!");delay(Interval);
  digitalWrite(processing, HIGH);Serial.println("processing ON!");delay(Interval);
//  digitalWrite(connection, LOW);Serial.println("Connection OFF!");delay(Interval);
  digitalWrite(processing, LOW);Serial.println("processing OFF!");delay(Interval);
}
