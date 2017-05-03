
#include <Wire.h>


//volatile unsigned int counterRight = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile float counter = 0;

void setup() {
  Serial.begin (9600);
  Wire.begin(9);
  pinMode(2, INPUT);           // set pin to input
  pinMode(3, INPUT);
  //pinMode(auxPinLeft, INPUT);
  // set pin to input
  digitalWrite(2, HIGH);       // turn on pullup resistors
  digitalWrite(3, HIGH);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, CHANGE);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, CHANGE);  //left
  Wire.onRequest(requestEvent);   // fucntion to run when asking for data
}

void loop() {
  // Create message
  String strR = String(counter);
  String strF = strR + '/';

  Wire.write(strF.c_str());
  Serial.println (strF);
  delay(500);
}

void requestEvent() {

//  // Create message
//  String strR = String(counter);
//  String strF = strR + '/';
//
//  Wire.write(strF.c_str());
//  Serial.println (strF);
//  counter = 0;
}

void ai0() {
  if ( digitalRead(3) == 0 ) {
    if ( digitalRead(2) == 0 ) {
      // A fell, B is low
      counter--; // moving reverse
    } else {
      // A rose, B is low
      counter++; // moving forward
    }
  } else {
    if ( digitalRead(2) == 0 ) {
      // A fell, B is high
      counter++; // moving forward
    } else {
      // A rose, B is high
      counter--; // moving reverse
    }
  }
}

void ai1() {
  if ( digitalRead(2) == 0 ) {
    if ( digitalRead(3) == 0 ) {
      // A fell, B is low
      counter--; // moving reverse
    } else {
      // A rose, B is low
      counter++; // moving forward
    }
  } else {
    if ( digitalRead(3) == 0 ) {
      // A fell, B is high
      counter++; // moving forward
    } else {
      // A rose, B is high
      counter--; // moving reverse
    }
  }
}

