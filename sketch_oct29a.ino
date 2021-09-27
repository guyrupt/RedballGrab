/*
  Arduino Slave for Raspberry Pi Master
  i2c_slave_ard.ino
  Connects to Raspberry Pi via I2C
  
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
// Include the Wire library for I2C
#include <Wire.h>
 
// LED on pin 13
const int ledPin = 13; 
 
void setup() {
  // Join I2C bus as slave with address 8
  Wire.begin(0x8);
  Serial.begin(9600);
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off
  pinMode(ledPin, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(ledPin, LOW);
}
 
// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    int c = Wire.read(); // receive byte as a character
    Serial.println(c);
    if(c==1){
      digitalWrite(11,  LOW);
      digitalWrite(6,  LOW);
      digitalWrite(10, HIGH);
      digitalWrite(5, HIGH);
    }
    else if(c==2){
      digitalWrite(11, LOW);
      digitalWrite(6, LOW);
      digitalWrite(10, LOW);
      digitalWrite(5, LOW);
    }
    else if(c==3){
      digitalWrite(10, LOW);
      digitalWrite(5, LOW);
      digitalWrite(11, HIGH);
      digitalWrite(6, HIGH);
    }
    else if (c==4){
      digitalWrite(11,  LOW);
      digitalWrite(6, LOW);
      digitalWrite(10, HIGH);
      digitalWrite(5, LOW);
    }
    else if (c==5){
      digitalWrite(11,  LOW);
      digitalWrite(6,  LOW);
      digitalWrite(10,LOW);
      digitalWrite(5, HIGH);
    }
  }
}
void loop() {
  delay(100);
}
