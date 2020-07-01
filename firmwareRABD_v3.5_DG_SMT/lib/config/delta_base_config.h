#ifndef DELTA_BASE_CONFIG_H
#define DELTA_BASE_CONFIG_H

#define RABD    //DG2
//uncomment the base you're building
#define DELTA_BASE DIFF_2WD

//uncomment the motor driver you're using
#define USE_TROY_DRIVER//USE_L298_DRIVER
//#define USE_ORIENTAL_DRIVER//USE_L298_DRIVER
//#define USE_BTS7960_DRIVER

//uncomment the IMU you're using
//#define USE_GY85_IMU
#define USE_MPU6050_IMU

#define DEBUG 0
#define selftest 0

#define BLV   //TROY   BLV

//=================ROBOT SPEC ()=============================
 

#ifdef DG2    //define your robot' specs here      一圈10000count   減速比30    一圈39.27公分
  #define K_P 2   // P constant  0.6   //2
  #define K_I 5   // I constant  0.3   //5
  #define K_D 0.5 // D constant  0.5    //0.5  
  #define MAX_RPM 133//4000 // motor's maximum RPM
  #define COUNTS_PER_REV 10000 // wheel encoder's no of ticks per rev
  #define WHEEL_DIAMETER 0.15 // wheel's diameter in meters
  #define PWM_BITS 8 // PWM Resolution of the microcontroller
  #define BASE_WIDTH 0.5 // width of the plate you are using   27  31  35   //!!2wd輪距需除以2
#endif 

#ifdef RABD   //define your robot' specs here      一圈10000count   減速比30    一圈47.12公分
  
  #define MAX_RPM 133//4000 // motor's maximum RPM  //150
  #define COUNTS_PER_REV 10000 // wheel encoder's no of ticks per rev
  #define WHEEL_DIAMETER 0.15 // wheel's diameter in meters
  #define PWM_BITS 8 // PWM Resolution of the microcontroller
  #define BASE_WIDTH 0.47 // width of the plate you are using  //0.53
#endif 

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 22//15
#define MOTOR1_ENCODER_B 23//14 

#define MOTOR2_ENCODER_A 14//11
#define MOTOR2_ENCODER_B 15//12 

#define MOTOR3_ENCODER_A 20//17
#define MOTOR3_ENCODER_B 21//16 

#define MOTOR4_ENCODER_A 16//9
#define MOTOR4_ENCODER_B 17//10

#ifdef USE_TROY_DRIVER
#define MOTOR_DRIVER NC2
#endif 

#define IMU_PUBLISH_RATE 200 //hz    10/50/80
#define VEL_PUBLISH_RATE 200 //hz    10/50/80
#define Driver_Modbus_Query_RATE 1 //hz   
#define COMMAND_RATE 20 //hz        15
#define DEBUG_RATE 5

//high byte&low byte define
#define LO(a)	((unsigned char) (0xFF&(a)))
#define HI(a)	((unsigned char) (((unsigned int) (a)) >> 8))

#endif
