//master
#include <Wire.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <PID_v1.h>

// Control pins
#define PWM_RIGHT_PIN           5
#define PWM_LEFT_PIN            6
#define DIR_RIGHT_PIN           7
#define DIR_LEFT_PIN            8

char t[10]={};//empty array where to put the numbers comming from the slave
char t2[10]={};//empty array where to put the numbers comming from the slave
volatile int Val; // varible used by the master to sent data to the slave
float pulsesRight;
float pulsesLeft;
float angVelRight;

double Kpr=0, Kir=0, Kdr=0;
double cmd_rVel, VelRight, rOutput;
PID rightPID(&VelRight, &rOutput, &cmd_rVel,Kpr,Kir,Kdr, DIRECT);

double Kpl=0, Kil=0, Kdl=0;
double cmd_lVel, VelLeft, lOutput;
PID leftPID(&VelLeft, &lOutput, &cmd_lVel,Kpl,Kil,Kdl, DIRECT);

unsigned long time;
unsigned long newtime = 0;
unsigned long oldtime = 0;

unsigned long time2;
unsigned long newtime2 = 0;
unsigned long oldtime2 = 0;

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  pinMode(DIR_LEFT_PIN , OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(PWM_LEFT_PIN , OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);

  cmd_rVel=10;
  cmd_lVel=10;

  rightPID.SetOutputLimits(0, 255);
  leftPID.SetOutputLimits(0, 255);
  //turn the PID on
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
}

void loop() {
  Wire.requestFrom(8, 10);    // request 3 bytes from slave device #8

  //gathers data comming from slave
  int i=0; //counter for each bite as it arrives
  while (Wire.available()) { 
    t[i] = Wire.read(); // every character that arrives it put in order in the empty array "t"
    if(t[i]=='/')
    {
      break;
    }
    i=i+1;
  }
    Wire.requestFrom(9, 10);    // request 3 bytes from slave device #8

  //gathers data comming from slave
  int i2=0; //counter for each bite as it arrives
  while (Wire.available()) { 
    t[i2] = Wire.read(); // every character that arrives it put in order in the empty array "t"
    if(t2[i2]=='/')
    {
      break;
    }
    i2=i2+1;
  }
  
  int length = strlen(t);
  t[length-1] = '\0'; // retira a '/' da mensagem
  pulsesRight= atoi(t); // transformar num número

  int length2 = strlen(t2);
  t2[length2-1] = '\0'; // retira a '/' da mensagem
  pulsesLeft= atoi(t2); // transformar num número
  
  //calculo da velocidade (RPM) right
  newtime = millis();
  time = newtime - oldtime;
  VelRight=(pulsesRight*60*1000)/(400*time);
  oldtime=newtime;
  Serial.print("RPM Right: ");
  Serial.println(VelRight);
  t[0]='\0';

  //calculo da velocidade (RPM) left
  newtime2 = millis();
  time2 = newtime2 - oldtime2;
  VelLeft=(pulsesLeft*60*1000)/(400*time);
  oldtime2=newtime2;
  Serial.print("RPM Left: ");
  Serial.println(VelLeft);
  t[0]='\0';

  //implementação do PID
  rightPID.Compute();
  analogWrite(PWM_RIGHT_PIN,rOutput);
  Serial.print("Output Right: ");
  Serial.println(rOutput);
  leftPID.Compute();
  analogWrite(PWM_LEFT_PIN,rOutput);
  Serial.print("Output Left: ");
  Serial.println(lOutput);

  //direção
  if(cmd_rVel<0)
  {digitalWrite(DIR_RIGHT_PIN,HIGH);  }
  if(cmd_lVel<0)
  {digitalWrite(DIR_LEFT_PIN,HIGH);  }

  delay(50); //give some time to relax
}
