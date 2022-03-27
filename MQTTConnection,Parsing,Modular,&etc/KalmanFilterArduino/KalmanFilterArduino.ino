float SensorData,KalmanFilterData;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt, R, Q;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  R = 100; Q = 0.01; Pt_prev = 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  SensorData = analogRead(A0);
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + ( Kt * (SensorData - Xt_update));
  Pt = (1-Kt) * Pt_update;
  Xt_prev = Xt;
  Pt_prev = Pt;
  KalmanFilterData = Xt;
  Serial.print(SensorData,3);
  Serial.print(",");
  Serial.print(KalmanFilterData,3);
  Serial.println();
  
  delayMicroseconds(100);
}
