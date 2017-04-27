//master
#include <Wire.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

class PID{
public:
  
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  
  double setPoint;
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  
  double process(){
    // Implementação PID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

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
double VelLeft;

unsigned long time2;
unsigned long newtime2 = 0;
unsigned long oldtime2 = 0;

/*/////////////////////////////////////////////////////////////////
 *                      ROS
//////////////////////////////////////////////////////////////////*/

ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist pulses;

ros::Publisher pub("data_nardo", &pulses);

PID meuPid(1.8, 0, 0);

void updateVel(const geometry_msgs::Twist& cmd_vel)
{
  //update velocide desejada
  meuPid.setSetPoint(msg.linear.x);
}

ros::Subscriber <geometry_msgs::Twist> sub("/nardo/cmd_vel", updateVel);

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  pinMode(DIR_LEFT_PIN , OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(PWM_LEFT_PIN , OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

}

int lOutput = 50;

void loop() {
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

  meuPid.addNewSample(VelLeft);
  pulses.linear.x = VelLeft;
  pub.publish(&pulses);
  
  lOutput = (meuPid.process());
  Serial.println(lOutput);
  if(lOutput > 200)
  {
    lOutput=200;
  }
  Serial.print("Output Left: ");
  Serial.println(lOutput);

  analogWrite(PWM_LEFT_PIN,lOutput);


  delay(100); //give some time to relax
}
