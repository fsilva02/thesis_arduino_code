#include <Wire.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PWM_LEFT_PIN            6
#define DIR_LEFT_PIN            8

char t[10]={};//empty array where to put the numbers comming from the slave
char t2[10]={};//empty array where to put the numbers comming from the slave
volatile int Val; // varible used by the master to sent data to the slave
float pulsesRight;
float pulsesLeft;
float angVelRight;
double VelLeft;
unsigned long time2;
unsigned long newtime2 = 0;
unsigned long oldtime2 = 0;

/*/////////////////////////////////////////////////////////////////
 *                      ROS
//////////////////////////////////////////////////////////////////*/

ros::NodeHandle nh;
geometry_msgs::Twist pulses;

ros::Publisher pub("data_nardo", &pulses);

void GoGo(const geometry_msgs::Twist& cmd_vel)
{
  analogWrite(PWM_LEFT_PIN,200);
}

ros::Subscriber <geometry_msgs::Twist> sub("/nardo/cmd_vel", GoGo);


void setup() {
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master)

  pinMode(DIR_LEFT_PIN , OUTPUT);
  pinMode(PWM_LEFT_PIN , OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  analogWrite(PWM_LEFT_PIN,0);

}

void loop() {
  nh.spinOnce();
  // put your main code here, to run repeatedly:
  Wire.requestFrom(9, 10);    // request 3 bytes from slave device #8

  //gathers data comming from slave
  int i2=0; //counter for each bite as it arrives
  while (Wire.available()) { 
    t2[i2] = Wire.read(); // every character that arrives it put in order in the empty array "t"
    if(t2[i2]=='/')
    {
      break;
    }
    i2=i2+1;
  }
  
  int length2 = strlen(t2);
  t2[length2-1] = '\0'; // retira a '/' da mensagem
  pulsesLeft= atoi(t2); // transformar num número
  Serial.print("Pulses Left: ");
  Serial.println(pulsesLeft);

  //calculo da velocidade (RPM) left
  newtime2 = millis();
  time2 = newtime2 - oldtime2;
  VelLeft=(pulsesLeft/400)/(time2*0.00001666666667);
  oldtime2=newtime2;
  Serial.print("RPM Left: ");
  Serial.println(VelLeft);
  t[0]='\0';

  pulses.linear.x = abs(VelLeft);
  pub.publish(&pulses);

  delay(50); //give some time to relax

}
