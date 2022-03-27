String A = "wheel1/power_meter/";
int32_t Board = 68203;
String C = "/calibration/pubs1";
String _topicS;
char topicSubscribe[2048];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i<1; i++){
    _topicS = A;
    _topicS += String(Board);
    _topicS += C; 
  }
  _topicS.toCharArray(topicSubscribe,2048);
  Serial.println(topicSubscribe);
  delay(1000);
}
