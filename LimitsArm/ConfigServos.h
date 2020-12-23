/**
 ************************************************************************************
 * @file     ConfigServos.h
 * @brief    NAME header file
 * @version  V1.0.0
 * @date     XX-XXXX-20XX
 *************************************************************************************
 */
#ifndef  __CONFIGSERVOS_H
#define __CONFIGSERVOS_H

#ifdef __cplusplus
extern "C" {
#endif
/*************************************************************************************/
#include <stdint.h>
/*************************************************************************************/
/*Variables*/
/*************************************************************************************/
#define OTHER 0x666
#define MiTime  2000
#define LED_USER  13
#define MIN_PWM  130
#define MAX_PWM  570
#define FREQUENCY 50
#define INIT_POS_WAIST 90
#define INIT_POS_SHOULDER 70
#define INIT_POS_ELBOWN 80
#define INIT_POS_GRIPPER 20
/*************************************************************************************/
#define SERVO_WAIST_PIN       4      //Cintura = q1
#define SERVO_WAIST_OFFSET    0
#define SERVO_WAIST_MIN_POS   0
#define SERVO_WAIST_MAX_POS 180
/*************************************************************************************/
#define SERVO_SHOULDER_PIN       5   //Hombro = q2
#define SERVO_SHOULDER_OFFSET    0
#define SERVO_SHOULDER_MIN_POS  60
#define SERVO_SHOULDER_MAX_POS 180
/*************************************************************************************/
#define SERVO_ELBOW_PIN       6      //Codo = q3
#define SERVO_ELBOW_OFFSET    0
#define SERVO_ELBOW_MIN_POS  10
#define SERVO_ELBOW_MAX_POS 110
/*************************************************************************************/
#define SERVO_GRIPPER_PIN       7    //Pinza = q4
#define SERVO_GRIPPER_OFFSET    0
#define SERVO_GRIPPER_MIN_POS  20
#define SERVO_GRIPPER_MAX_POS  60
/*************************************************************************************/
typedef struct{
  uint8_t pin;
  int offset;
  int min_pos;
  int max_pos;
}RobotServo_t;
/********************************************************************************************************************************************************/
void writeServo(RobotServo_t &servo, int angle);
void writeServoSlow(RobotServo_t &servo, int angle);
void detachServo(RobotServo_t &servo);
void ResetServos(RobotServo_t &servo);
/********************************************************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __CONFIGSERVOS_H */
