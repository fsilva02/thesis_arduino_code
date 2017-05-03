//master
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>


char t[5]={};//empty array where to put the numbers comming from the slave
int Pulses;

int Kp=0, Ki=0, Kd=0;
double cmd_Vel, Vel, Output;
PID PID(&Vel, &Output, &cmd_Vel,Kp,Ki,Kd, DIRECT);

double time;
double newtime = 0;
double oldtime = 0;

/*/////////////////////////////////////////////////////////////////
 *                      ROS
//////////////////////////////////////////////////////////////////*/

ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist pidCons;
geometry_msgs::Twist pulses;

//publicação das rotações
ros::Publisher pub("data_nardo", &pulses);

void updateVel(const geometry_msgs::Twist& cmd_vel)
{
  //update velocide desejada
  cmd_Vel= msg.linear.x;
}

void defineCons(const geometry_msgs::Twist& pidCons)
{
  Kp = pidCons.linear.x;
  Ki = pidCons.linear.y;
  Kd = pidCons.linear.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("/nardo/cmd_vel", updateVel);
ros::Subscriber <geometry_msgs::Twist> sub2("/nardo/pid", defineCons);

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  pinMode(8 , OUTPUT); // direção
  pinMode(6 , OUTPUT); //pwm

  cmd_Vel=6;
  
  PID.SetOutputLimits(0, 220);
  //turn the PID on
  PID.SetMode(AUTOMATIC);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub);
}

void loop() {
  Wire.requestFrom(9, 10);    // request 3 bytes from slave device #8

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
  Vel=(Pulses*60*1000)/(400*time);
  oldtime=newtime;
  Serial.print("RPM Right: ");
  Serial.println(Vel);
  t[0]='\0';
  pulses.linear.x = Vel;
  
  pub.publish(&pulses);

  //implementação do PID
  PID.Compute();
  analogWrite(6,Output);
  Serial.print("Output Right: ");
  Serial.println(Output);
  
  //direção
  if(cmd_Vel<0)
  {digitalWrite(8,HIGH);  }

  nh.spinOnce();
  
  delay(50); //give some time to relax
}
