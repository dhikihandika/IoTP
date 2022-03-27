float peak = 0;
float peakBefore = 0;
float vmax,vrms = 0;
float fc = 0.88582677165354330708661417322835; //When constant calibration y = 225 v

int number[1000];
int number1[1000];
int number2[1000];
float arrData[1000];

int countNumSame[sizeof(number) / sizeof(int)];
int lengthVar,largestCount,modest;
int h,i,j,k,l,d,e = 0;
bool isCalculated = false;

void setup(){
  Serial.begin(9600);
  lengthVar = sizeof(number) / sizeof(int);
  isCalculated = false;
  Serial.println(" ");
  Serial.println("Start!");
}

void calculate(){
  //Serial.print("Panjang DataArray: ");Serial.println(lengthVar);
  for(h = 0; h <= (lengthVar - 1); h++){
    for( i = 0; i <= (lengthVar - 1); i++){
      //Serial.print("Data yg di-calculated: ");Serial.println(number[j]);
      if(number1[j] == number2[i]){
        countNumSame[j]++;
      }
    }
    if( i >= (lengthVar - 1)){
      j++;
      i = 0;
    }
  }
  if(h >= (lengthVar - 1)){
    isCalculated = true;
    for(k = 0; k <= (lengthVar - 1); k++){
      Serial.print(countNumSame[k]);
    }
    Serial.println(" ");
  } 
}

void findMode(){
  Serial.print("Cetak Data Yg dikerjakan #");Serial.print(d);Serial.print(": ");
  for (int k = 0; k <= (lengthVar - 1); k++){
    Serial.print(number[k]);Serial.print("|");
  }
  Serial.println("");
  if(!isCalculated){
    calculate();
  } else {
    if(k >= (lengthVar - 1)){
      for(l = 0; l <= (lengthVar - 1);  l++){
        //Serial.print("Data Array Saat False: ");Serial.println(number[l]);
        //Serial.print("Nilai Modest Saat False: ");Serial.println(modest);
        if(countNumSame[l] > largestCount){
          largestCount = countNumSame[l];
          modest = number[l];
        }
      }
      e++;
    }
    Serial.print("Largest Count:");Serial.println(largestCount);
    Serial.print("Modest:");Serial.println(modest);
    //Serial.print("Masuk Else:");Serial.println(e);
    isCalculated = false;
    Serial.print("Data Array:");
    for(int m = 0; m <= (lengthVar - 1); m++){
//      number[m] = arrData[m];
//      number1[m] = arrData[m];
//      number2[m] = arrData[m];
      countNumSame[m]=0;
//      Serial.print(number[m]);Serial.print("|");
//      Serial.print(number[m]);Serial.print("|");
      arrData[m]=0;
    }
    Serial.println("");
    Serial.println("_______________________________________");
    Serial.print("D:");Serial.println(d);
    d=0;e=0;j=0;largestCount=0;
  }
}

void voltageMeasure(){
  peak = analogRead(A0);
  if(peak>peakBefore){
    peakBefore = peak;
    vmax = peak;
//    Serial.println(vmax);
  }
  if(peak<peakBefore){
    peakBefore = peak;
    vrms = ((0.244*vmax)-744.32); // equation of two lines
    vrms = vrms * fc; 
  }
}

void loop(){
  voltageMeasure();
  if(d<=(lengthVar)){
    number[d-1] = vrms;
    number1[d-1] = vrms;
    number2[d-1] = vrms;
    arrData[d-1] = vrms;
//    Serial.print("vrms: #");Serial.print(d-1);Serial.print(": ");Serial.println(vrms);
  }
  if(d>=(lengthVar)){
    findMode();
  };
  d++;
}
