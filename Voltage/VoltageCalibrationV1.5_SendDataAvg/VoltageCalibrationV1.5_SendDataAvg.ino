#include <SimpleKalmanFilter.h>

float peak = 0;
float peakBefore = 0;
float vmax, vrms = 0;
float fc = 0.88582677165354330708661417322835;
float Buffer;
int g = 0;

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
    Buffer += vrms * fc; 
    Serial.println(vrms * fc);
    if(g >= 1000){
      Serial.print("Avg:");Serial.println(Buffer/g);
      Buffer = 0; g = 0;
    }
    g++;
  }
}

void loop() {
  voltageMeasure();
}
