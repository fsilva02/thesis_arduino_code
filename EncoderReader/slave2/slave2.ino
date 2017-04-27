
#include <Wire.h>


//volatile unsigned int counterRight = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile float counterR = 0;
volatile float counterL = 0;

void setup() {
  Serial.begin (9600);
  Wire.begin(8);
  pinMode(2, INPUT);           // set pin to input
  pinMode(3, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT); 
  //pinMode(auxPinLeft, INPUT); 
  // set pin to input
  digitalWrite(2, HIGH);       // turn on pullup resistors
  digitalWrite(3, HIGH);       // turn on pullup resistors
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);  //right
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);  //left
  Wire.onRequest(requestEvent);   // fucntion to run when asking for data
}

void loop() {
    // Create message
  String strR = String(counterR);
  String strL = String(counterL);
  String strF = strR + 'x'+ strL + '/';
  
  Wire.write(strF.c_str());
  Serial.println (strF);
  counterR = 0;
  counterL= 0;
  delay(1000);
}
void requestEvent() {


 
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(6)==LOW) {
    counterR++;
  }else{
    counterR--;
  }
  Serial.println("hi");
}

void ai1() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(7)==LOW) {
    counterL++;
  }else{
    counterL--;
  }
}


