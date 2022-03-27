float xn = 0;
float prev_xn = 0;
float prev_pn = 0;
float kn = 0;
float rn = 0;
float Q = 0.026;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  prev_xn = analogRead(A0);
  prev_pn = prev_xn + random(-10000,5000)/100;
}

void loop() {
  // put your main code here, to run repeatedly:
  float zn = analogRead(A0);
  float rn = zn + random(-10000,5000)/100;
  kn = prev_pn/(prev_pn + rn); // Find a kalman gain 
  xn = (prev_xn + (kn*(zn + prev_xn))); 
  float pn = (((1 - kn) * prev_pn ) + ((prev_xn - zn) * Q));
  prev_xn = xn;
  prev_pn = pn;
  Serial.println(pn);
}
