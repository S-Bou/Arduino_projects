#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PWM 130
#define MAX_PWM 570

#define GRIPPER_CLOSED 25
#define GRIPPER_OPENED 50
#define JOINTS 3

#define TIMEgriper 50
#define TIMEuno  1
#define TIMEdos  2
#define TIMEtres 3

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

RobotPosition_t pT0={params.l0+params.l1+params.l3+params.l5,-params.d5,params.h1+params.l2}; //{152.1,-5.0,144.5}
RobotPosition_t pT1={90.0,-90.0, 160.0};
RobotPosition_t pT2={120.0,-120.0, 135.0};
RobotPosition_t pT3={140.0, 100.0, 90.0};
RobotPosition_t pT4={150.0, 120.0, 60.0};
RobotPosition_t pT5={190.0, -30.0, 100.0};
RobotPosition_t pT6={200.0, -30.0, 70.0};

RobotPosition_t pT0L={90.0, 90.0, 90.0};
RobotPosition_t pT1L={45.0, 90.0, 90.0};

typedef struct 
{
  uint8_t pin;
  int offset;
  int min_pos;
  int max_pos;
} RobotServo_t;

RobotServo_t robotServos[JOINTS]={{4,0,0,180},{5,2,50,170},{6,-5,30,120}};
RobotServo_t robotGripper={7,0,30,50};

//TODO: Define several configurations
double qN[JOINTS]={0.0,0.0,0.0};       /*Vector global que contiene los ángulos q1, q2 y q3, se actualiza en moveJ*/
double q0[JOINTS]={90.0,90.0,90.0};
double p0[JOINTS]={152.1,-5.0,144.5};
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  //Initializes the PCA9685 servo driver
  pwm.begin();
  pwm.setPWMFreq(50);
  PORTB=0x02; //Salida para led de error
  DDRB=0x00;  //Inicializar a nivel bajo la salida para el led
  //Sets the servo to the initial position
  for (int i=0;i<JOINTS;i++){writeServo(robotServos[i],(int)q0[i]);}
  //moveL(robotServos, p0, pT1L, TIMEuno, params);

}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop()
{
  PickAndPlace();
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void writeServo(const RobotServo_t &servo, int angle)
{
  angle=constrain(angle+servo.offset,servo.min_pos,servo.max_pos);
  
  int pulse_width;
  pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
  pwm.setPWM(servo.pin,0,pulse_width);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void detachServo(const RobotServo_t &servo)
{
  pwm.setPWM(servo.pin,0,0);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void moveAbsJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const double qt[JOINTS], const double T)
{
  double a[JOINTS],b[JOINTS],c[JOINTS],d[JOINTS],q,t,t0;
  //TODO: Compute trajectory parameters for each joint
  for(int i=0;i<JOINTS;i++)
  {
    a[i]=-2*(qt[i]-q0[i])/pow(T,3);
    b[i]=3*(qt[i]-q0[i])/pow(T,2);
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
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void moveJ(const RobotServo_t robotServos[JOINTS],const double q0[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params)
{
  Serial.print("qT={ X, Y, Z} = {");Serial.print(target.x);
  Serial.print(", "); Serial.print(target.y);
  Serial.print(", "); Serial.print(target.z);Serial.println("} Coords");  
  double qT[JOINTS];
  inverseKin(target, params, qT);
  Serial.print("qN={q1,q2,q3} = {");Serial.print(qT[0]);
  Serial.print(", "); Serial.print(qT[1]);
  Serial.print(", "); Serial.print(qT[2]);Serial.println("} Degrees");  
  /*Comprobación de que los cálculos dan valores correctos*/
  DataError(qT);
  moveAbsJ(robotServos,q0,qT,T);
  /*Actulizar el vector global "qN" con la posición actual*/
  for(int i=0;i<JOINTS;i++)
  {
    qN[i]=qT[i];
  }
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void inverseKin(const RobotPosition_t &target,const RobotParams_t &params, double *q)
{
  double q1=atan2(target.x-params.l0, -target.y)+asin(params.d5/sqrt(pow(target.x-params.l0,2)+pow(target.y,2)));
  q[0]=q1*180/M_PI;
  double pw[JOINTS]={target.x+(-params.l5*sin(q1)-params.l0),target.y+(params.l5*cos(q1)),target.z+0};
  double r=sqrt(pow(pw[0],2)+pow(pw[1],2))-params.l1;
  double ze=pw[2]-params.h1;
  double alfa=atan(ze/r);
  double s=sqrt(pow(r,2)+pow(ze,2));
  double gamma=acos((pow(params.l2,2)+pow(params.l3,2)-pow(s,2))/(2*params.l2*params.l3));
  double beta=acos((pow(s,2)+pow(params.l2,2)-pow(params.l3,2))/(2*s*params.l2));
  double q2=M_PI-alfa-beta;
  q[1]=q2*180/M_PI;
  double q3_0=M_PI-gamma;
  double e=sqrt(pow(params.l3O,2)+pow(params.l2,2)-2*params.l3O*params.l2*cos(q3_0));
  double psi=asin((params.l3O*sin(q3_0))/e);
  double phi=acos((pow(e,2)+pow(params.l3I,2)-pow(params.l4,2))/(2*e*params.l3I));
  double q3=psi+phi+M_PI/2-q2;
  q[2]=q3*180/M_PI;
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void moveL(const RobotServo_t robotServos[JOINTS], const double q0L[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params)
{ 
  double qC[JOINTS],a[JOINTS],b[JOINTS],c[JOINTS],d[JOINTS],s[JOINTS],t,t0;
  /*TODO: Call forwardKin to compute the current gripper position from the given configuration.*/
  forwardKin(qC, params, target);
  Serial.print("qC={ X, Y, Z} = {");Serial.print(qC[0]);
  Serial.print(", "); Serial.print(qC[1]);
  Serial.print(", "); Serial.print(qC[2]);Serial.println("} Coordenadas");  
 
 /*Then compute a target position and move the gripper along the trajectory by sending proper servos commands using the inverse kinematics.*/ 
  t0=millis()/1000.0;
  t=0.0;
  while(t<T)
  {
    double Pt_P0=sqrt(pow(qC[0]-q0L[0],2)+pow(qC[1]-q0L[1],2)+pow(qC[2]-q0L[2],2));    
    for(int i=0;i<JOINTS;i++)
    {
      a[i]=-2*Pt_P0/pow(T,3);
      b[i]=-3*Pt_P0/pow(T,2);
      c[i]=0.0;
      d[i]=0.0;
    }
    for (int i=0;i<JOINTS;i++)
    {
      qC[i]=a[i]*pow(t,3)+b[i]*pow(t,2)+c[i]*t+d[i];
    }
    forwardKin(qC, params, target);
    Serial.print("s={ X, Y, Z} = {");Serial.print(s[0]);
    Serial.print(", "); Serial.print(s[1]);
    Serial.print(", "); Serial.print(s[2]);Serial.println("} Coordenadas");     
    delay(20);
    t=millis()/1000.0-t0;
  } 
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void forwardKin(double *q, const RobotParams_t &params, RobotPosition_t target)
{
//TODO: Return in q the values of the configuration for the given target position.

  double q1=target.x*M_PI/180;
  double q2=target.y*M_PI/180;
  double q3=target.z*M_PI/180;

  double phi= q3+q2-(M_PI/2);
  double f=sqrt(pow(params.l3I,2)+pow(params.l2,2)-2*params.l3I*params.l2*cos(phi));
  double tau=asin((params.l3I*sin(phi))/f);
  double mu=acos((pow(f,2)+pow(params.l3O,2)-pow(params.l4,2))/(2*f*params.l3O));
  double q3_0=tau+mu; 
  double r=-params.l2*cos(q2)-params.l3*cos(q2+q3_0);
  double ptx=params.l0+(r+params.l1+params.l5)*sin(q1)-params.d5*cos(q1);
  q[0]=ptx;
  double pty=-(r+params.l1+params.l5)*cos(q1)-params.d5*sin(q1);
  q[1]=pty;
  double ptz=params.h1+params.l2*sin(q2)+params.l3*sin(q2+q3_0);
  q[2]=ptz;
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void OpenGripper(void)
{ 
  for(int i=GRIPPER_CLOSED;i<GRIPPER_OPENED;i++)
  {
    writeServo(robotGripper, i);
    delay(TIMEgriper);
  }
  detachServo(robotGripper);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void CloseGripper(void)
{
  for(int i=GRIPPER_OPENED;i>GRIPPER_CLOSED;i--)
  {
    writeServo(robotGripper, i);
    delay(TIMEgriper);
  }
  detachServo(robotGripper);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
  /*Comprobación de que los cálculos dan valores correctos, si se obtiene " nan=Not A Number "  
  volvemos a la posición inicial y bucle infinito con led de error*/
void DataError(double qX[])
{
  for(int i=0;i<JOINTS;i++)
  {
    if (isnan(qX[i]))
    {
      moveJ(robotServos, qN, pT0, TIMEdos, params);
      Serial.println("ERROR: posicionamiento no válido");
      for (int i=0;i<JOINTS;i++){detachServo(robotServos[i]);}
      detachServo(robotGripper);
      for(;;)    // Led intermitente infinito
      {
         DDRB=0x02;
         delay(100);
         DDRB=0x00;
         delay(100);
      }  
    }
  }
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void PickAndPlace(void)
{
  moveJ(robotServos, q0, pT1, TIMEuno, params);
  moveJ(robotServos, qN, pT2, TIMEdos, params);
  CloseGripper();
  moveJ(robotServos, qN, pT1, TIMEuno, params);  
  moveJ(robotServos, qN, pT0, TIMEdos, params);

  moveJ(robotServos, qN, pT3, TIMEdos, params);
  moveJ(robotServos, qN, pT4, TIMEuno, params);
  OpenGripper();
  moveJ(robotServos, qN, pT3, TIMEuno, params);
  moveJ(robotServos, qN, pT0, TIMEdos, params);
  
  moveJ(robotServos, qN, pT5, TIMEuno, params);
  moveJ(robotServos, qN, pT6, TIMEdos, params);
  CloseGripper();
  moveJ(robotServos, qN, pT0, TIMEuno, params);
  
  moveJ(robotServos, qN, pT1, TIMEuno, params);
  moveJ(robotServos, qN, pT2, TIMEdos, params);
  OpenGripper();
  moveJ(robotServos, qN, pT1, TIMEuno, params);
  moveJ(robotServos, qN, pT3, TIMEtres, params);
  moveJ(robotServos, qN, pT4, TIMEdos, params);
  CloseGripper();
  moveJ(robotServos, qN, pT3, TIMEuno, params);
  moveJ(robotServos, qN, pT5, TIMEuno, params);
  moveJ(robotServos, qN, pT6, TIMEdos, params);
  OpenGripper();
  moveJ(robotServos, qN, pT0, TIMEuno, params);

  for (int i=0;i<JOINTS;i++){detachServo(robotServos[i]);}
  detachServo(robotGripper);
  delay(2000);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
