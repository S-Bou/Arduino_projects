#include <Adafruit_PWMServoDriver.h>

//#define ROBOT_EMULATION
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define JOINTS  4
#define MIN_PWM  130 // This is the 'minimum' pulse length count (out of 4096)
#define MAX_PWM  570 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

typedef struct 
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

#ifdef ROBOT_EMULATION
RobotServo_t robotServos[JOINTS]={{1,0,0,180},{2,0,50,170},{3,0,20,160},{4,0,60,90}};
#else
RobotServo_t robotServos[JOINTS]={{4,0,0,180},{5,0,60,180},{6,0,20,120},{7,0,20,60}};
#endif
double MOV_0[JOINTS]={ 90.0, 90.0, 90.0, 40.0};
double MOV_1[JOINTS]={ 35.0, 70.0, 35.0, 60.0};
double MOV_2[JOINTS]={145.0, 70.0,100.0, 90.0};
double MOV_3[JOINTS]={145.0, 70.0, 35.0, 60.0};

double MOV_P1[JOINTS]={ 60.0,  90.0, 90.0, 20.0};
double MOV_P2[JOINTS]={ 60.0, 170.0, 90.0, 20.0};
double MOV_P3[JOINTS]={ 60.0, 170.0, 90.0, 60.0};
double MOV_P4[JOINTS]={ 60.0, 170.0, 40.0, 60.0};

double MOV_P5[JOINTS]={ 60.0, 170.0, 40.0, 20.0};
double MOV_P6[JOINTS]={ 90.0,  90.0, 90.0, 20.0};
double MOV_P7[JOINTS]={ 60.0, 170.0, 90.0, 60.0};
double MOV_P8[JOINTS]={ 60.0, 170.0, 40.0, 60.0};

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_0[i]);} delay(3000);
//  for (int i=0;i<JOINTS;i++) writeServo(robotServos[i],(int)MOV_1[i]); delay(3000);
//  for (int i=0;i<JOINTS;i++) writeServo(robotServos[i],(int)MOV_2[i]); delay(3000);
//  for (int i=0;i<JOINTS;i++) writeServo(robotServos[i],(int)MOV_3[i]); delay(3000);
//  for (int i=0;i<JOINTS;i++) writeServo(robotServos[i],(int)MOV_0[i]); delay(3000);
//
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P1[i]);} delay(1000);
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P2[i]);} delay(1000);
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P3[i]);} delay(1000);
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P4[i]);} delay(1000);
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P5[i]);} delay(1000);
//  for (int i=0;i<JOINTS;i++) {writeServo(robotServos[i],(int)MOV_P6[i]);} delay(3000);
  for (int i=0;i<JOINTS;i++) detachServo(robotServos[i]);

}
 
void loop() { 

}

void writeServo(const RobotServo_t &servo, int angle)
{
  angle=constrain(angle+servo.offset,servo.min_pos,servo.max_pos);
  #ifndef ROBOT_EMULATION
    int pulse_width;
    angle=constrain(angle, servo.min_pos, servo.max_pos);
    pulse_width=map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    pwm.setPWM(servo.pin, 0, pulse_width);
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
  pwm.setPWM(servo.pin, 0, 0);
  #endif
}
