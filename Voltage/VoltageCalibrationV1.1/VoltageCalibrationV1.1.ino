//#define CAL_LITERATION 1 //literation for calibration
//
//bool is_calibrate;

float peak = 0;
float peakBefore = 0;
float vmax,vmin,vrms = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  peak = analogRead(A0);
//  Serial.println(peak);
  if(peak>peakBefore){
    peakBefore = peak;
    vmax = peak;
//    Serial.println(vmax);
  }
  if(peak<peakBefore){
    peakBefore = peak;
    vrms = (0.2395 * vmax) - 760.37; // equation of two lines           
    Serial.println(vrms);
  }
}
