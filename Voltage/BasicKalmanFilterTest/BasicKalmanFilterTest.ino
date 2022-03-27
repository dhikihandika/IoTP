#include <SimpleKalmanFilter.h>

float peak, peak1, peak2, peak3 = 0;
float peakBefore, peakBefore1, peakBefore2, peakBefore3 = 0;
float vmax,vrms, vmax1,vrms1, vmax2,vrms2, vmax3,vrms3 = 0;
float fc = 0.88582677165354330708661417322835;

/*
 This sample code demonstrates how to use the SimpleKalmanFilter object. 
 Use a potentiometer in Analog input A0 as a source for the reference real value.
 Some random noise will be generated over this value and used as a measured value.
 The estimated value obtained from SimpleKalmanFilter should match the real
 reference value.

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  Serial.begin(115200);
}

void voltageMeasure(){
  float real_value = analogRead(A0);
  float measured_value = real_value + random(-100,100)/100.0;
  float peak = simpleKalmanFilter.updateEstimate(measured_value);
  if(peak>peakBefore){
    peakBefore = peak;
    vmax = peak;
//    Serial.println(vmax);
  }
  if(peak<peakBefore){
    peakBefore = peak;
    vrms = ((0.244*vmax)-744.32); // equation of two lines
    vrms = vrms * fc; 
    Serial.println(vrms);
  }
}

void loop() {
voltageMeasure();

}
