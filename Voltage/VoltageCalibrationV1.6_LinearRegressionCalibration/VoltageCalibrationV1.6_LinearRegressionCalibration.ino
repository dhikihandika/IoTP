#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
float peakRS,peakST,peakTR = 0;
float peakBeforeRS,peakBeforeST,peakBeforeTR = 0;
float vmaxRS,vmaxST,vmaxTR = 0;
float vRS,vST,vTR = 0;
float bufferVRS, bufferVST, bufferVTR = 0;
float MvAvgRS, MvAvgST, MvAvgTR = 0;
float V380RS, V380ST, V380TR = 0;
float Vrms380 = 390.00;     // Take form measurement tools
float Vrms220 = 230.00;     // Take from measurement tools
int a,b,c;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("Start!");
}
void voltageMeasure(){
  float real_valueRS = analogRead(A0);
  float real_valueST = analogRead(A3);
  float real_valueTR = analogRead(A6);
  float measured_valueRS = real_valueRS + random(-100,100)/100.0;
  float measured_valueST = real_valueST + random(-100,100)/100.0;
  float measured_valueTR = real_valueTR + random(-100,100)/100.0;
  peakRS = simpleKalmanFilter.updateEstimate(measured_valueRS);
  peakST = simpleKalmanFilter.updateEstimate(measured_valueST);
  peakTR = simpleKalmanFilter.updateEstimate(measured_valueTR);
  //MeasureRS
  if(peakRS>peakBeforeRS){
    peakBeforeRS = peakRS;
    vmaxRS = peakRS;
  }
  if(peakRS<peakBeforeRS){
    peakBeforeRS = peakRS;
    vRS = ((0.2395 * vmaxRS) - 760.37); //Liner Regression y = 0,2395x - 760,37
    bufferVRS += vRS; 
    if(a >= 1000){
      MvAvgRS = bufferVRS/a;
      V380RS = (Vrms380 * MvAvgRS)/Vrms220;
      if(MvAvgRS < 0){
        MvAvgRS = 0;
      }
      bufferVRS = 0; a = 0;
    }
    a++;
  }
  //MeasureST
  if(peakST>peakBeforeST){
    peakBeforeST = peakST;
    vmaxST = peakST;
  }
  if(peakST<peakBeforeST){
    peakBeforeST = peakST;
    vST = ((0.2395 * vmaxST) - 760.37); // Liner Regression
    bufferVST += vST; 
    if(b >= 1000){
      MvAvgST = bufferVST/b;
      V380ST = (Vrms380 * MvAvgST)/Vrms220;
      if(MvAvgST < 0){
        MvAvgST = 0;
      }
      bufferVST = 0; b = 0;
    }
    b++;
  }
  //MeasureTR
  if(peakTR>peakBeforeTR){
    peakBeforeTR = peakTR;
    vmaxTR = peakTR;
  }
  if(peakTR<peakBeforeTR){
    peakBeforeTR = peakTR;
    vTR = ((0.2395 * vmaxTR) - 760.37); // Liner Regression
    bufferVTR += vTR; 
    if(c >= 1000){
      MvAvgTR = bufferVTR/c;
      V380TR = (Vrms380 * MvAvgTR)/Vrms220;
      if(MvAvgTR < 0){
        MvAvgTR = 0;
      }
      bufferVTR = 0; c = 0;
    }
    c++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  voltageMeasure();
  Serial.print(vRS);Serial.print(",");Serial.print(vST);Serial.print(",");Serial.println(vTR);
//  Serial.print(V380RS);Serial.print(",");Serial.print(V380ST);Serial.print(",");Serial.println(V380TR);
}
