float peak1,peak2,peak3 = 0;
float peakBefore = 0;
float vmax,vrms = 0;
float fc = 0.88582677165354330708661417322835; //When constant calibration y = 225 v

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("Start!");
}

int voltageMeasure(){
  
}

void loop() {
  // put your main code here, to run repeatedly:
  peak1 = analogRead(A0);
  peak2 = analogRead(A3);
  peak3 = analogRead(A6);
//  if(peak>peakBefore){
//    peakBefore = peak;
//    vmax = peak;
////    Serial.println(vmax);
//  }
//  if(peak<peakBefore){
//    peakBefore = peak;
//    vrms = ((0.244*vmax)-744.32); // equation of two lines
//    vrms = vrms * fc; 
//    Serial.println(peak1);
    Serial.print(peak1);Serial.print(",");Serial.print(peak2);Serial.print(",");Serial.println(peak3);
//  }
}
