#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "ConfigServos.h"
/********************************************************************************************************************/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
RobotServo_t servoWaist = {SERVO_WAIST_PIN, SERVO_WAIST_OFFSET, SERVO_WAIST_MIN_POS, SERVO_WAIST_MAX_POS};
RobotServo_t servoShoulder = {SERVO_SHOULDER_PIN, SERVO_SHOULDER_OFFSET, SERVO_SHOULDER_MIN_POS, SERVO_SHOULDER_MAX_POS};
RobotServo_t servoElbow = {SERVO_ELBOW_PIN, SERVO_ELBOW_OFFSET, SERVO_ELBOW_MIN_POS, SERVO_ELBOW_MAX_POS};
RobotServo_t servoGripper = {SERVO_GRIPPER_PIN, SERVO_GRIPPER_OFFSET, SERVO_GRIPPER_MIN_POS, SERVO_GRIPPER_MAX_POS};
uint8_t CURRENT_POS_WAIST = INIT_POS_WAIST-1;
uint8_t CURRENT_POS_SHOULDER = INIT_POS_SHOULDER-1;
uint8_t CURRENT_POS_ELBOWN = INIT_POS_ELBOWN-1;
uint8_t CURRENT_POS_GRIPPER = INIT_POS_GRIPPER-1;
/********************************************************************************************************************/
void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

//  CenterArm();
//  delay(2000);
//  ShowLimits();
//  delay(2000);
//  CenterArm();

//  writeServoSlow(servoShoulder, 140);
//  delay(1000);
//  writeServoSlow(servoElbow, 40);
//  delay(1000);
//  writeServoSlow(servoShoulder, 180);
//  delay(1000);
//  writeServoSlow(servoShoulder, 180);
//  delay(1000);
}
/********************************************************************************************************************/
void loop() {

  writeServoSlow(servoWaist, 0);
  delay(1000);
  writeServoSlow(servoWaist, 180);
  delay(1000);
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void writeServo(RobotServo_t &servo, int angle)
{
  int pulse_width;
  angle = constrain(angle, servo.min_pos, servo.max_pos);  //Restringe el rango de ángulos a los límites
  pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
  pwm.setPWM(servo.pin, 0, pulse_width);
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void detachServo(RobotServo_t &servo)
{
  pwm.setPWM(servo.pin, 0, 0);
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void writeServoSlow(RobotServo_t &servo, int angle)
{
  int pulse_width;
  angle = constrain(angle, servo.min_pos, servo.max_pos);

  switch (servo.pin)
  {
    case SERVO_WAIST_PIN: 
          if(angle > CURRENT_POS_WAIST)
          {
            for(uint8_t i=CURRENT_POS_WAIST; i<angle; i++)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }else{
            for(uint8_t i=CURRENT_POS_WAIST; i>angle; i--)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }
          CURRENT_POS_WAIST = angle;
          break;
          
    case SERVO_SHOULDER_PIN: 
          if(angle > CURRENT_POS_SHOULDER)
          {
            for(uint8_t i=CURRENT_POS_SHOULDER; i<angle; i++)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }else{
            for(uint8_t i=CURRENT_POS_SHOULDER; i>angle; i--)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }
          CURRENT_POS_SHOULDER = angle;
          break;
          
    case SERVO_ELBOW_PIN: 
          if(angle > CURRENT_POS_ELBOWN)
          {
            for(uint8_t i=CURRENT_POS_ELBOWN; i<angle; i++)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }else{
            for(uint8_t i=CURRENT_POS_ELBOWN; i>angle; i--)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }
          CURRENT_POS_ELBOWN = angle;
          break;
    case SERVO_GRIPPER_PIN: 
          if(angle > CURRENT_POS_GRIPPER)
          {
            for(uint8_t i=CURRENT_POS_GRIPPER; i<angle; i++)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }else{
            for(uint8_t i=CURRENT_POS_GRIPPER; i>angle; i--)
            {
              pulse_width = map(i+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
              pwm.setPWM(servo.pin, 0, pulse_width);
              delay(25);
            }
          }
          CURRENT_POS_GRIPPER = angle;
          break;
    default:
         break;
  }
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void ResetServos(RobotServo_t &servo)
{
  switch (servo.pin)
  {
    case SERVO_WAIST_PIN: 
          writeServoSlow(servoWaist,INIT_POS_WAIST);
          //detachServo(servoWaist);
          break;
          
    case SERVO_SHOULDER_PIN: 
          writeServoSlow(servoShoulder,INIT_POS_SHOULDER);
          //detachServo(servoShoulder);
          break;
          
    case SERVO_ELBOW_PIN: 
          writeServoSlow(servoElbow,INIT_POS_ELBOWN);
          //detachServo(servoElbow);
          break;
          
    case SERVO_GRIPPER_PIN: 
          writeServoSlow(servoGripper,INIT_POS_GRIPPER);
          //detachServo(servoGripper);
          break;
          
    default:
          writeServoSlow(servoWaist,INIT_POS_WAIST);
          writeServoSlow(servoShoulder,INIT_POS_SHOULDER);
          writeServoSlow(servoElbow,INIT_POS_ELBOWN);
          writeServoSlow(servoGripper,INIT_POS_GRIPPER);
          break;
  }
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void ShowLimits(void)
{
  int MyTime = 1000;
  writeServoSlow(servoWaist, 0);
  delay(MyTime);
  writeServoSlow(servoWaist, 180);
  delay(MyTime); 
  ResetServos(servoWaist);
  delay(MyTime);
  writeServoSlow(servoShoulder, 0);
  delay(MyTime);
  writeServoSlow(servoShoulder, 180);
  delay(MyTime); 
  ResetServos(servoShoulder);
  delay(MyTime);
  writeServoSlow(servoElbow, 0);
  delay(MyTime);
  writeServoSlow(servoElbow, 180);
  delay(MyTime); 
  ResetServos(servoElbow);
  delay(MyTime);
  writeServoSlow(servoGripper, 0);
  delay(MyTime);
  writeServoSlow(servoGripper, 180);
  delay(MyTime); 
  ResetServos(servoGripper);
  delay(MyTime);
}
/********************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void CenterArm(void)
{
  ResetServos(servoWaist);
  detachServo(servoWaist);
  ResetServos(servoShoulder);
  detachServo(servoShoulder);
  ResetServos(servoElbow);
  detachServo(servoElbow);
  ResetServos(servoGripper);
  detachServo(servoGripper);
}
/********************************************************************************************************************/
