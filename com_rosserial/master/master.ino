//master
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>


char t[4]={};//empty array where to put the numbers comming from the slave
int Pulses;

int Kpr=0, Kir=0;
double cmd_rVel, VelRight, rOutput;
PID rightPID(&VelRight, &rOutput, &cmd_rVel,Kpr,Kir,0, DIRECT);

int Kpl=0, Kil=0;
double cmd_lVel, VelLeft, lOutput;
PID leftPID(&VelLeft, &lOutput, &cmd_lVel,Kpl,Kil,0, DIRECT);

double time;
double newtime = 0;
double oldtime = 0;

unsigned long newtime2 = 0;
unsigned long oldtime2 = 0;

/*/////////////////////////////////////////////////////////////////
 *                      ROS
//////////////////////////////////////////////////////////////////*/

ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist pidCons;
geometry_msgs::Twist pulses;


ros::Publisher pub("data_nardo", &pulses);

void updateVel(const geometry_msgs::Twist& cmd_vel)
{
  //update velocide desejada
  cmd_rVel= msg.linear.x;
  cmd_lVel= msg.linear.y;
}

void defineCons(const geometry_msgs::Twist& pidCons)
{
  //definição das constantes a usar
  Kpr = pidCons.linear.x;
  Kir = pidCons.linear.y;
//  Kdr = pidCons.linear.z;
  Kpl = pidCons.angular.x;
  Kil = pidCons.angular.y;
  //Kdl = pidCons.angular.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("/nardo/cmd_vel", updateVel);
ros::Subscriber <geometry_msgs::Twist> sub2("/nardo/pid", defineCons);

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  pinMode(8 , OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6 , OUTPUT);
  pinMode(5, OUTPUT);

  cmd_rVel=6;
  cmd_lVel=6;

  rightPID.SetOutputLimits(0, 255);
  leftPID.SetOutputLimits(0, 255);
  //turn the PID on
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);

  nh.initNode();
  //nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub);
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

  t[strlen(t)-1] = '\0'; // retira a '/' da mensagem
  Pulses= atoi(t); // transformar num número

  //calculo da velocidade (RPM) right
  newtime = millis();
  time = newtime - oldtime;
  VelRight=(Pulses*60*1000)/(400*time);
  oldtime=newtime;
  Serial.print("RPM Right: ");
  Serial.println(VelRight);
  t[0]='\0';
  pulses.linear.x = VelRight;
  
  Wire.requestFrom(9, 10);    // request 3 bytes from slave device #8

  //gathers data comming from slave
  i=0; //counter for each bite as it arrives
  while (Wire.available()) { 
    t[i] = Wire.read(); // every character that arrives it put in order in the empty array "t"
    if(t[i]=='/')
    {
      break;
    }
    i=i+1;
  }
  
  t[strlen(t)-1] = '\0'; // retira a '/' da mensagem
  Pulses= atoi(t); // transformar num número

  //calculo da velocidade (RPM) left
  newtime2 = millis();
  time = newtime2 - oldtime2;
  VelLeft=(Pulses*60*1000)/(400*time);
  oldtime2=newtime2;
  Serial.print("RPM Left: ");
  Serial.println(VelLeft);
  t[0]='\0';
  
  pulses.linear.y = VelLeft;
  pub.publish(&pulses);

  //implementação do PID
  rightPID.Compute();
  analogWrite(5,rOutput);
  Serial.print("Output Right: ");
  Serial.println(rOutput);
  leftPID.Compute();
  analogWrite(6,rOutput);
  Serial.print("Output Left: ");
  Serial.println(lOutput);

  //direção
  if(cmd_rVel<0)
  {digitalWrite(7,HIGH);  }
  if(cmd_lVel<0)
  {digitalWrite(8,HIGH);  }

  delay(50); //give some time to relax
}
