String dataIn = "$0.69;0.70;0.46;0.66;";
String dt[32];
float y_irms[32];
int i;

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  delay(2000);
  parsingData();
  dataIn="";
}

void loop() {
  // put your main code here, to run repeatedly:

}

void parsingData(){
  int j=0;
  
  //kirim data yang telah diterima sebelumnya
  Serial.print("in_data: ");
  Serial.println(dataIn);
   
  //inisialisasi variabel, (reset isi variabel)
  dt[j]="";
  //proses parsing data
  for(i=1; i<dataIn.length(); i++){
    //pengecekan tiap karakter dengan karakter (#) dan (,)
    if((dataIn[i] == '$') || (dataIn[i] == ';')){
    //increment variabel j, digunakan untuk merubah index array penampung
    j++;
    dt[j]="";       //inisialisasi variabel array dt[j]
    }else{
    //proses tampung data saat pengecekan karakter selesai.
    dt[j] = dt[j] + dataIn[i];
    }
  }

  for (int k = 0; k <= 3; k++) {
    y_irms[k] = dt[k].toFloat();
    Serial.println(y_irms[k]);
  }
//  //kirim data hasil parsing
//  Serial.print("data 1 : ");
//  Serial.println(dt[0].toFloat());
//  Serial.print("data 2 : ");
//  Serial.println(dt[1].toFloat());
//  Serial.print("data 3 : ");
//  Serial.println(dt[2].toFloat());
//  Serial.print("data 4 : ");
//  Serial.println(dt[3].toFloat());
//  Serial.print("\n\n");
}
