#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
float peakR,peakS,peakT = 0;
float peakBeforeR,peakBeforeS,peakBeforeT = 0;
float vmaxR,vmaxS,vmaxT = 0;
float vrmsR,vrmsS,vrmsT = 0;
float bufferVR, bufferVS, bufferVT = 0;
float MvAvgR, MvAvgS, MvAvgT = 0;
float fc = 0.88582677165354330708661417322835;
int a,b,c;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("Start!");
}
void voltageMeasure(){
  float real_valueR = analogRead(A0);
  float real_valueS = analogRead(A3);
  float real_valueT = analogRead(A6);
  float measured_valueR = real_valueR + random(-100,100)/100.0;
  float measured_valueS = real_valueS + random(-100,100)/100.0;
  float measured_valueT = real_valueT + random(-100,100)/100.0;
  float peakR = simpleKalmanFilter.updateEstimate(measured_valueR);
  float peakS = simpleKalmanFilter.updateEstimate(measured_valueS);
  float peakT = simpleKalmanFilter.updateEstimate(measured_valueT);
  //MeasureR
  if(peakR>peakBeforeR){
    peakBeforeR = peakR;
    vmaxR = peakR;
  }
  if(peakR<peakBeforeR){
    peakBeforeR = peakR;
    vrmsR = ((0.244 * vmaxR) - 744.32); // equation of two lines
    bufferVR += vrmsR * fc; 
    if(a >= 1000){
      MvAvgR = bufferVR/a;
      bufferVR = 0; a = 0;
    }
    a++;
  }
  //MeasureS
  if(peakS>peakBeforeS){
    peakBeforeS = peakS;
    vmaxS = peakS;
  }
  if(peakS<peakBeforeS){
    peakBeforeS = peakS;
    vrmsS = ((0.244 * vmaxS) - 744.32); // equation of two lines
    bufferVS += vrmsS * fc; 
    if(b >= 1000){
      MvAvgS = bufferVS/b;
      bufferVS = 0; b = 0;
    }
    b++;
  }
  //MeasureT
  if(peakT>peakBeforeT){
    peakBeforeT = peakT;
    vmaxT = peakT;
  }
  if(peakT<peakBeforeT){
    peakBeforeT = peakT;
    vrmsT = ((0.244 * vmaxT) - 744.32); // equation of two lines
    bufferVT += vrmsT * fc; 
    if(c >= 1000){
      MvAvgT = bufferVT/c;
      bufferVT = 0; c = 0;
    }
    c++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  voltageMeasure();
  Serial.print(MvAvgT);Serial.print(",");Serial.print(MvAvgS);Serial.print(",");Serial.println(MvAvgR);
}
