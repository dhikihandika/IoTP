float a[4]={1,2,3,4};
float b[4]={5,6,7,8};
float c[8];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

int i,j,k = 0;
void loop() {
  // put your main code here, to run repeatedly:
  for(i = 0; i<=3; i++){
    c[i] = a[i];
    c[i+4] = b[i];
  }
//  for(k = 0; k<=3; k++){
//    c[k+4] += b[k];
//  }
  for(j =0; j<=8; j++){
    Serial.println(c[j]);
  }
  delay(1000);
}
