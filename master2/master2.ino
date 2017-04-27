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
volatile int Val; // varible used by the master to sent data to the slave
float pulsesRight;
float angVelRight;

double Kp=1, Ki=0.05, Kd=0.25;
double cmd_rVel, VelRight, rOutput;
PID rightPID(&VelRight, &rOutput, &cmd_rVel,Kp,Ki,Kd, DIRECT);

unsigned long time;
unsigned long newtime = 0;
unsigned long oldtime = 0;

ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist pidCons;

//Declaring String variable
std_msgs::Float32 str_msg;

//Defining Publisher
ros::Publisher pub("data_nardo", &str_msg);

void CallBack(const geometry_msgs::Twist& cmd_vel)
{
  //update velocide desejada
  cmd_rVel= msg.linear.x;
}

void defineCons(const geometry_msgs::Twist& pidCons)
{
  //definição das constantes a usar
  Kp = pidCons.linear.x;
  Ki = pidCons.linear.y;
  Kd = pidCons.linear.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("/nardo/cmd_vel", CallBack);
ros::Subscriber <geometry_msgs::Twist> sub2("/nardo/pid", defineCons);

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub);

  pinMode(DIR_LEFT_PIN , OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(PWM_LEFT_PIN , OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);

  rightPID.SetOutputLimits(0, 255);
  //turn the PID on
  rightPID.SetMode(AUTOMATIC);
}

void loop() {
  nh.spinOnce();
  
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
  
  int length = strlen(t);
  t[length-1] = '\0'; // retira a '/' da mensagem
  pulsesRight= atoi(t); // transformar num número
  
  //calculo da velocidade
  newtime = millis();
  time = newtime - oldtime;
  angVelRight= (pulsesRight*2*3.1416)/(time*1200);
  VelRight=angVelRight*0.165;
  oldtime=newtime;
  
  //publicação da leitura
  str_msg.data=VelRight;
  pub.publish(&str_msg);
  delay(50); //give some time to relax
  t[0]='\0';

  //implementação do PID
  rightPID.Compute();
  analogWrite(PWM_RIGHT_PIN,rOutput);

  //direção
  if(cmd_rVel<0)
  {digitalWrite(DIR_RIGHT_PIN,HIGH);  }
}
