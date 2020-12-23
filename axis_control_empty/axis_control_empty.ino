//#define ROBOT_EMULATION //Uncomment to use the emulated version in CoppeliaSim (does not move the real robot)
#define PCA9685  //Uncomment to use the PCA9685 servo driver instead of the Servo library

#ifdef PCA9685
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  #define MIN_PWM 130
  #define MAX_PWM 570
#else
  #include <Servo.h>
  Servo servos[12]; //Max number of digital signals
#endif

#define GRIPPER_CLOSED 20
#define GRIPPER_OPENED 50
#define JOINTS 3

typedef struct
{
double l0;
double h1;
double l1;
double l2;
double l3;
double l3I;
double l3O;
double l4;
double l5;
double d5;
} RobotParams_t;
RobotParams_t params={5.0,64.5,15.1,80.0,80.0,23.9,35.0,80.0,52.0,5.0};

typedef struct
{
double x;
double y;
double z;
} RobotPosition_t;
RobotPosition_t pT={params.l0+params.l1+params.l3+params.l5,-params.d5,params.h1+params.l2}; //{152.1,-5,144.5}

typedef struct 
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

#ifdef ROBOT_EMULATION
RobotServo_t robotServos[JOINTS]={{1,0, 0,180},{2,0,50, 170},{3,0,20,160}};
#else
RobotServo_t robotServos[JOINTS]={{4,0,0,180},{5,2,50,170},{6,-5,20,120}};
RobotServo_t robotGripper={7,0,20,50};
#endif
//TODO: Define several configurations
double q0[JOINTS]={ 90.0, 90.0, 90.0};
double q1[JOINTS]={ 90.0,170.0, 30.0};
double q2[JOINTS]={ 90.0,170.0, 30.0};
double q3[JOINTS]={ 90.0, 90.0, 90.0};
//void moveAbsJ(const RobotServo_t servos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], const double T);
#define TIME 3
void setup() {
  Serial.begin(115200);
  #ifndef ROBOT_EMULATION
    #ifdef PCA9685
      //Initializes the PCA9685 servo driver
//      Wire.pins(0,2);
//      Wire.begin(0,2);
      pwm.begin();
      pwm.setPWMFreq(50);
    #endif
  #endif
  moveJ(robotServos, q0, pT, TIME, params);
}

void loop() { 
  //Sets the servo to the initial position
//  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q0[i]);}
//  delay(100);
//  //TODO: Call moveAbsJ to do some movements
//  moveAbsJ(robotServos,q0,q1,TIME);
//  moveAbsJ(robotServos,q1,q2,TIME);
//  moveAbsJ(robotServos,q2,q3,TIME);
//  moveAbsJ(robotServos,q3,q0,TIME); 
//  for (int i=0;i<JOINTS;i++){detachServo(robotServos[i]);}
//  delay(1000);

//  writeServo(robotGripper, 30);
//  delay(2000);
//  writeServo(robotGripper, 50);
//  delay(2000);
}

void writeServo(const RobotServo_t &servo, int angle)
{
  angle=constrain(angle+servo.offset,servo.min_pos,servo.max_pos);
  #ifndef ROBOT_EMULATION
    #ifdef PCA9685
      int pulse_width;
      pulse_width   = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
      pwm.setPWM(servo.pin,0,pulse_width);
    #else
      servos[servo.pin].attach(servo.pin);
      servos[servo.pin].write(angle);
    #endif
  #else
    Serial.print("S");
    Serial.print(servo.pin);
    Serial.print(":");
    Serial.print((int)angle,DEC);
    Serial.println(";");
  #endif
}

void detachServo(const RobotServo_t &servo)
{
  #ifndef ROBOT_EMULATION
    #ifdef PCA9685
      pwm.setPWM(servo.pin,0,0);
    #else
      servos[servo.pin].detach();
    #endif
  #endif
}

void moveAbsJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], const double T)
{
  double a[JOINTS],b[JOINTS],c[JOINTS],d[JOINTS],q,t,t0;
  //TODO: Compute trajectory parameters for each joint
  for(int i=0;i<JOINTS;i++)
  {
    a[i]=-2*(qT[i]-q0[i])/pow(T,3);
    b[i]=3*(qT[i]-q0[i])/pow(T,2);
    c[i]=0.0;
    d[i]=q0[i];
  }
  t0=millis()/1000.0;
  t=0.0;
  while(t<T)
  {
    for (int i=0;i<JOINTS;i++)
    {
      //TODO: Evaluate the trajectory for joint "i" at time "t" and store the result in "q".
      q=a[i]*pow(t,3)+b[i]*pow(t,2)+c[i]*t+d[i];
      writeServo(robotServos[i],(int)q);
    }
    delay(20);
    t=millis()/1000.0-t0;
  }
}

void moveJ(const RobotServo_t robotServos[JOINTS],const double q0[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params)
{
  double qT[JOINTS];
  inverseKin(target, params, qT);
  //moveAbsJ(robotServos,q0,qT,T);
  Serial.print("q1=");Serial.print(qT[0]*180/M_PI);
  Serial.print("; q2="); Serial.print(qT[1]*180/M_PI);
  Serial.print("; q3="); Serial.println(qT[2]*180/M_PI);
}

void inverseKin(const RobotPosition_t &target,const RobotParams_t &params, double *q)
{
  q[0]= (atan2(target.x-params.l0, -target.y)+asin(params.d5/sqrt(pow(target.x-params.l0,2)+pow(target.y,2))));
  double pw[JOINTS]={target.x+(-params.l5*sin(q[0])-params.l0),target.y+(params.l5*cos(q[0])),target.z+0};
  double r=sqrt(pow(pw[0],2)+pow(pw[1],2))-params.l1;
  double ze=pw[2]-params.h1;
  double alfa=atan(ze/r);
  double s=sqrt(pow(r,2)+pow(ze,2));
  double gamma=acos((pow(params.l2,2)+pow(params.l3,2)-pow(s,2))/(2*params.l2*params.l3));
  double beta=acos((pow(s,2)+pow(params.l2,2)-pow(params.l3,2))/(2*s*params.l2));
  q[1]=M_PI-alfa-beta;
  double q3_0=M_PI-gamma;
  double e=sqrt(pow(params.l3,2)+pow(params.l2,2)-2*params.l3*params.l2*cos(q3_0));
  double psi=asin((params.l3*sin(q3_0))/e);
  double phi=acos((pow(e,2)+pow(params.l2,2)-pow(params.l4,2))/(2*e*params.l3));
  q[2]=psi+phi+M_PI/2-q[1];
}
