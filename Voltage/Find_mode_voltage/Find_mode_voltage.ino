int number[] = {220,221,221,221,24,221,98,221,220,78,221,220,221,220};
int number1[] = {220,221,221,221,24,221,98,221,220,78,221,220,221,220};
int number2[] = {220,221,221,221,24,221,98,221,220,78,221,220,221,220};

int countNumSame[sizeof(number) / sizeof(int)];
int lengthVar,largestCount,modest;
int h,i,j,k,l = 0;
int d,e = 1;
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
    //Serial.print("Data Array:");
    for (int m = 0; m < 14; m++){
      number[m]++;
      number1[m]++;
      number2[m]++;
      countNumSame[m]=0;
      //Serial.print(number[m]);Serial.print("|");
    }
    Serial.println("_______________________________________");
    Serial.print("D:");Serial.println(d);
    d=1;e=1;j=0;largestCount=0;
    //Serial.println("");
    //Serial.println("----------------------------");
    //delay(500);
  }
}

void loop(){
  findMode();
  d++;
}
