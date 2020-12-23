
//#define ROBOT_EMULATION //Uncomment to use the emulated version in CoppeliaSim (does not move the real robot)
#define PCA9685  //Uncomment to use the PCA9685 servo driver instead of the Servo library

#define TIME 1000

#ifdef PCA9685
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  #define MIN_PWM 130
  #define MAX_PWM 570
#else
  #include <Servo.h>
  Servo servos[12]; //Max number of digital signals
#endif

#define JOINTS  4

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
RobotServo_t robotServos[JOINTS]={{4,0,0,180},{5,0,50,170},{6,0,20,160},{7,0,20,60}};
#endif

double q0[JOINTS]={90.0,90.0,90.0,30.0};
double q1[JOINTS]={90.0,90.0,90.0,60.0};
double q2[JOINTS]={90.0,90.0,90.0,20.0};

//TODO: Define several configurations

//void moveAbsJ(const RobotServo_t servos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], const double T);

void setup() {
  DDRD =  0b10000000; //Pin 13 output LED
  PORTD = 0b10000000; //Pin 13 up level
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

  //Sets the servo to the initial position
  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q0[i]);}

  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q1[i]);}
  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q2[i]);}
  
  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q0[i]);}
  
  delay(3000);
  //TODO: Call moveAbsJ to do some movements
  
  delay(3000);
  for (int i=0;i<JOINTS;i++)
    detachServo(robotServos[i]);
}
 
void loop() { 
  
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
  t0=millis()/1000.0;
  t=0.0;
  while(t<T)
  {
    for (int i=0;i<JOINTS;i++)
    {
      //TODO: Evaluate the trajectory for joint "i" at time "t" and store the result in "q".
      writeServo(robotServos[i],(int)q);
    }
    delay(20);
    t=millis()/1000.0-t0;
  }
}
