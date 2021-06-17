#include <Arduino.h>
#include "Mux.h"
using namespace admux;

//incoming serial byte
unsigned int inByte = 0;
bool calibrating = false;
const int sensorsBase = 8; 
unsigned long tToSend = 0;
unsigned long tToCalibrate = 0, tWithoutFeet = 0;
const int numReadings = 10;
Mux muxAnalog(Pin(26, INPUT, PinType::Analog), Pinset(6,7,8,9));
Mux muxDigital(Pin(10, OUTPUT, PinType::Digital), Pinset(2,3,4,5));
int reading[sensorsBase*sensorsBase];
int readingMin[sensorsBase][sensorsBase];
int timer[sensorsBase][sensorsBase];
int num = 0;
float centerOfGravityX = 0;
float centerOfGravityY = 0;

void setup() {
  Serial.begin(57600);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(20,OUTPUT);
  delay(500);
  calibration();
  delay(500);
  //establishContact();
}

void loop() {
    digitalWrite(19,HIGH);
    if (Serial.available() > 0 && !calibrating){
      inByte = Serial.read();
      readAndStore();
      AvgProcess();
      //switch inByte
      if(inByte == 'A'){ 
        sendArray();
        processedSender();
      }
    }
    if (num < 4) {
        if (millis() - tWithoutFeet > 2000) {
            if (millis() - tToCalibrate > 15000) {
                calibrating = true;
                calibration();
                tToCalibrate = millis();
                calibrating = false;
            }
            tWithoutFeet = millis();
        }
    }
    
}

void calibration(){
  digitalWrite(19,LOW);
  digitalWrite(18,HIGH);
  preCalibration();
  //if (calibrating) sendArray();
  for (int cal=0;cal<numReadings;cal++){
    for (int i=0;i<sensorsBase;i++){
      for (int j =0; j < sensorsBase; j++){ 
        muxDigital.write(HIGH,j);
        int newRead = muxAnalog.read(i);
        muxDigital.write(LOW,j);
        if (newRead < readingMin[i][j]){
          readingMin[i][j] = newRead;
        }
      }
    }
  }
  digitalWrite(18,LOW);
  tToCalibrate = millis();
  
}

void readAndStore(){  
  for (int i=0;i<sensorsBase;i++) {
    for (int j =0; j < sensorsBase; j++){ 
      muxDigital.write(HIGH,j);
      
      int newRead = muxAnalog.read(i);
      reading[(i * sensorsBase) + j] = newRead - readingMin[i][j];
      muxDigital.write(LOW,j);    
    }             
  }
}

void AvgProcess(){
  int totalInfluence = 0;
  for (int i=0;i<sensorsBase;i++) {
    for (int j =0; j < sensorsBase; j++){ 
      totalInfluence += reading[(i * sensorsBase) + j];
    }
  }
  int averageInfluence = totalInfluence / (sensorsBase*sensorsBase);
  float x = 0;
  float y = 0;
  num = 0;
  for (int i=0;i<sensorsBase;i++) {
    for (int j =0; j < sensorsBase; j++){ 
        if (reading[(i * sensorsBase) + j] > averageInfluence +15){
          x += i;
          y += j;
          num += 1;
        }
    }
  }
  if (num < 7) {
    centerOfGravityX = 40;
    centerOfGravityY = 40;
  }
  else {
    centerOfGravityX = (x/num)*10;
    centerOfGravityY = (y/num)*10; 
  }
//    centerOfGravityX = (x/num)*10;
//    centerOfGravityY = (y/num)*10; 
}


void processedSender(){
  
  //tToSend = millis();


  byte data[3] = {0,0,0};
  data[0] = (byte)centerOfGravityX;
  data[1] = (byte)centerOfGravityY;
  data[2] = (byte)num;

    //Serial.println((String)data[0] + " , " + (String)data[1]);
    //Serial.println((String)data[2]);
    //return;
  
  Serial.write(data,3);
}
    
void sendArray(){
  byte dataArray[sensorsBase*sensorsBase];
  for (int i=0;i<sensorsBase;i++) {
    for (int j =0; j < sensorsBase; j++){
      dataArray[(i * sensorsBase) + j] = 
          (byte)constrain(reading[(i * sensorsBase) + j],0,255);
    }
  }
  Serial.write(dataArray,sensorsBase*sensorsBase);
  
}

void preCalibration(){
  for(int i=0;i<sensorsBase;i++){
    for(int j=0;j<sensorsBase;j++){
      readingMin[i][j] = 1023;
    }
  }
}

void establishContact() {
  digitalWrite(20,HIGH);
  while (Serial.available() <= 0) {
    Serial.print('1');   // send a capital 1
    delay(300);
  }
  digitalWrite(20,LOW);
}
