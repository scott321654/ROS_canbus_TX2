#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>
#include <Servo.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header files for sub-imu
#include "geometry_msgs/Vector3.h"
//header file for imu
#include "delta_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"
#include <FlexCAN.h>

#include "delta_msgs/Velocities.h"
#include "delta_msgs/BatteryState.h"
#include "delta_msgs/MLSState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

/*header file for Multithread*/
#include "TeensyThreads.h"

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B);

Motor motor1(Motor::MOTOR_DRIVER, COUNTS_PER_REV);
Motor motor2(Motor::MOTOR_DRIVER, COUNTS_PER_REV);
Motor motor3(Motor::MOTOR_DRIVER, COUNTS_PER_REV);
Motor motor4(Motor::MOTOR_DRIVER, COUNTS_PER_REV);

/*
float PWM_MAX = pow(2, PWM_BITS) - 1;
Kinematics kinematics(Kinematics::DELTA_BASE, MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);
*/

Kinematics kinematics(Kinematics::DELTA_BASE, MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH);

//callback function prototypes
ros::NodeHandle nh;

delta_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
delta_msgs::BatteryState battery_msg;
ros::Publisher battery_pub("battery_state", &battery_msg);
delta_msgs::MLSState mls_msg;
ros::Publisher mls_pub("mls_state", &mls_msg);

//varialbe for CAN
CAN_message_t inMsg, TxMsg;
//MLS
int LinePosition = 0;
int LineLevel = 0;
bool LineGood;
int LineQuanity = 0; //LCP

bool state_bit[8];
bool LCP_bit[8];
//bool sp_flag = 0;
//varialbe for BMS
bool isCharging = 0;
int SOC = 0, Temperature = 0, Rack_Voltage = 0, Current = 0;
char Relay_State = 0, System_State = 0;
uint8_t Rack_Warning_State = 0, Rack_Error_State = 0, BMU_Error_State = 0;
bool fully_Charged = 0, Charging = 0, BatteryOVP = 0, Battery_Polarity_Reversed = 0, Battery_Short = 0,
     Fault = 0, Over_Gap = 0, Receiver_OTP = 0, Transmitter_OTP = 0, Wireless_Communication_OK = 0;
//varialbe for Modbus
unsigned char modbus_cmd[20];
byte Rx_Buffer[7]; //7 byte

void setup()
{
    InitSerial();
    InitROS();
    //clear fault
    delay(100);
    WriteCANdeviceWirelessChargerIni();
    delay(100);
    WriteCANdeviceWirelessChargerIni();
    delay(100);
    WriteCANdeviceWirelessChargerIni();
    delay(100);
    //wait until ros connection work
    InitConnection();
    delay(500);
    //clear fault
    //delay(100);
    // WriteCANdeviceWirelessChargerIni();
    // delay(100);
    // WriteCANdeviceWirelessChargerIni();
    // delay(100);
    // WriteCANdeviceWirelessChargerIni();
    // delay(100);
}
void loop()
{
    static unsigned long publish_vel_time = 0;
    static unsigned long prev_debug_time = 0;
    static unsigned long prev_search_time = 0;

    ReadCANdevice(); //開啟讀取BMS電池

    if ((millis() - prev_search_time) >= (1000 / 100)) //50
    {
        publishBatteryState();
        publishMLSState();
        //stay communication with wireless charger
        WriteCANdeviceWirelessCharger();
        prev_search_time = millis();
    }

    //this block publishes velocity based on defined rate
    if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
    {
        publishVelocities();
        publish_vel_time = millis();
    }
    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if (DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}

void InitSerial()
{
    Serial.begin(115200); //虛擬終端機  傳輸時間69us/byte
    Can0.begin(500000);
}

void InitROS()
{
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.advertise(raw_vel_pub);
    nh.advertise(battery_pub);
    nh.advertise(mls_pub);
}

void InitConnection()
{
    if (!selftest)
    {
        while (!nh.connected())
        {
            nh.spinOnce();
            //stay communication with wireless charger
            WriteCANdeviceWirelessCharger();
            delay(10);
        }
        nh.loginfo("DeltaAMRBase CONNECTED");
        delay(1);
    }
}

void publishMLSState()
{
    mls_msg.position = LinePosition;
    mls_msg.linegood = LineGood;
    mls_msg.linequantity = LineQuanity;
    mls_pub.publish(&mls_msg);
}

void publishBatteryState()
{
    battery_msg.SOC = SOC;
    battery_msg.Rack_voltage = Rack_Voltage;
    battery_msg.is_charging = isCharging;
    battery_msg.errors_1 = Rack_Warning_State;
    battery_msg.errors_2 = Rack_Error_State;
    battery_msg.errors_3 = BMU_Error_State;
    battery_pub.publish(&battery_msg);
}
void publishVelocities()
{
    //encoder to rpm
    Kinematics::velocities vel;
    motor1.updateSpeed(motor1_encoder.read());
    motor2.updateSpeed(motor2_encoder.read());
    motor3.updateSpeed(motor3_encoder.read());
    motor4.updateSpeed(motor4_encoder.read());
    //calculate the robot's speed based on rpm reading from each motor and platform used.
    /*
    if(selftest)
        vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());
    else    vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());  
    */
    //rpm to vel
    vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());
    //pass velocities to publisher object
    raw_vel_msg.linear_x = vel.linear_x;
    raw_vel_msg.linear_y = vel.linear_y;
    raw_vel_msg.angular_z = vel.angular_z;
    raw_vel_pub.publish(&raw_vel_msg);
}

void printDebug()
{
    char buffer[50];
    sprintf(buffer, "Encoder FrontLeft: %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf(buffer, "Encoder FrontRight: %ld", motor2_encoder.read());
    nh.loginfo(buffer);
}
void GetCRC(byte *message, byte *CRC)
{
    ushort CRCFull = 0xFFFF;
    byte CRCHigh = 0xFF, CRCLow = 0xFF;
    char CRCLSB;

    for (int i = 0; i < (16) - 2; i++)
    {
        CRCFull = (ushort)(CRCFull ^ message[i]);

        for (int j = 0; j < 8; j++)
        {
            CRCLSB = (char)(CRCFull & 0x0001);
            CRCFull = (ushort)((CRCFull >> 1) & 0x7FFF);

            if (CRCLSB == 1)
                CRCFull = (ushort)(CRCFull ^ 0xA001);
        }
    }
    CRC[1] = CRCHigh = (byte)((CRCFull >> 8) & 0xFF);
    CRC[0] = CRCLow = (byte)(CRCFull & 0xFF);
}

void WriteCANdeviceWirelessChargerIni()
{
    TxMsg.len = 8;
    TxMsg.id = 0x190;
    //bit0-19   25V   //bit20   power on    //bit32-50  10A
    //電壓25V-25000
    //TxMsg.buf[0] = 0xA8;//0b10101000;
    //TxMsg.buf[1] = 0x61;//0b01100001;
    //電壓27V-27000
    //TxMsg.buf[0] = 0x78;//0b01111000;
    //TxMsg.buf[1] = 0x69;//0b01101001;
    //電壓28V-28000
    TxMsg.buf[0] = 0x60; //0b01111000;
    TxMsg.buf[1] = 0x6D; //0b01101001;
                         //power on &&　clearfaults
    TxMsg.buf[2] = 0x20; //0b00100000;
    //power on
    //TxMsg.buf[2] = 0x10;//0b00010000;
    //power off
    //TxMsg.buf[2] = 0x00;//0b00000000;
    TxMsg.buf[3] = 0x00; //0b00000000;
    //電流10A
    //TxMsg.buf[4] = 0x10;//0b00010000;
    //TxMsg.buf[5] = 0x27;//0b00100111;
    //電流20A=20000=4E20
    TxMsg.buf[4] = 0x20;
    TxMsg.buf[5] = 0x4E;
    TxMsg.buf[6] = 0x00; //0b00000000;
    TxMsg.buf[7] = 0x00; //0b00000000;
    Can0.write(TxMsg);
}

void WriteCANdeviceWirelessCharger()
{
    TxMsg.len = 8;
    TxMsg.id = 0x190;
    //bit0-19   25V   //bit20   power on    //bit32-50  10A
    //電壓25V-25000
    //TxMsg.buf[0] = 0xA8;//0b10101000;
    //TxMsg.buf[1] = 0x61;//0b01100001;
    //電壓27V-27000
    //TxMsg.buf[0] = 0x78;//0b01111000;
    //TxMsg.buf[1] = 0x69;//0b01101001;
    //電壓28V-28000
    TxMsg.buf[0] = 0x60; //0b01111000;
    TxMsg.buf[1] = 0x6D; //0b01101001;
    //power on &&　clearfaults
    //TxMsg.buf[2] = 0x30; //0b00110000;
    //power on
    TxMsg.buf[2] = 0x10; //0b00010000;
    //power off
    //TxMsg.buf[2] = 0x00;//0b00000000;
    TxMsg.buf[3] = 0x00; //0b00000000;
    //電流10A
    //TxMsg.buf[4] = 0x10;//0b00010000;
    //TxMsg.buf[5] = 0x27;//0b00100111;
    //電流20A=20000=4E20
    TxMsg.buf[4] = 0x20;
    TxMsg.buf[5] = 0x4E;
    TxMsg.buf[6] = 0x00; //0b00000000;
    TxMsg.buf[7] = 0x00; //0b00000000;
    Can0.write(TxMsg);
    threads.delay(13);
}

void ReadCANdevice()
{
    while (Can0.available())
    {
        Can0.read(inMsg);
        if (inMsg.id == 800) //0x320 wireless-charger
        {
            fully_Charged = inMsg.buf[0] & 0x01;
            Charging = (inMsg.buf[0] >> 1) & 0x01;
            BatteryOVP = (inMsg.buf[0] >> 2) & 0x01;
            Battery_Polarity_Reversed = (inMsg.buf[0] >> 3) & 0x01;
            Battery_Short = (inMsg.buf[0] >> 4) & 0x01;
            Fault = (inMsg.buf[0] >> 5) & 0x01;
            Over_Gap = (inMsg.buf[0] >> 6) & 0x01;
            Receiver_OTP = (inMsg.buf[0] >> 7) & 0x01;
            Transmitter_OTP = (inMsg.buf[1]) & 0x01;
            ;
            Wireless_Communication_OK = (inMsg.buf[1] >> 1) & 0x01;
            ;
        }
        if (inMsg.id == 1136) //0x470 battery I
        {
            SOC = inMsg.buf[0];
            Temperature = inMsg.buf[1];
            Rack_Voltage = (inMsg.buf[2] + (inMsg.buf[3] << 8)) / 1;
            Current = (inMsg.buf[4] + (inMsg.buf[5] << 8)) / 1;
        }
        if (inMsg.id == 1137) //0x471 battery II
        {
            Relay_State = inMsg.buf[0];
            Rack_Warning_State = inMsg.buf[1];
            Rack_Error_State = inMsg.buf[2];
            BMU_Error_State = inMsg.buf[3];
            System_State = inMsg.buf[4];
            if (System_State == 7)
                isCharging = 1;
            else
                isCharging = 0;
        }

        if (inMsg.id == 395) //0x180+ID(11)  MLS REAR
        {
            for (int i = 0; i <= 7; i++)
                LCP_bit[i] = (inMsg.buf[6] >> i) & 0x1;
            for (int i = 0; i <= 7; i++)
                state_bit[i] = (inMsg.buf[7] >> i) & 0x1;
            LineLevel = state_bit[1] + (state_bit[2] << 1) + (state_bit[3] << 2); //強度
            if (LineLevel > 2)
            {
                if (inMsg.buf[3] == 0)
                {
                    LinePosition = inMsg.buf[2];
                }
                else if (inMsg.buf[3] == 255)
                {
                    LinePosition = -(256 - inMsg.buf[2]); //LinePosition
                }
            }
        }
        //todo codesys implementation below
        // if (state_bit[0] == 1) //確認偵測到磁條
        // {
        //     mag_go = 1;
        // }

        LineQuanity = LCP_bit[0] + (LCP_bit[1] << 1) + (LCP_bit[2] << 2); //磁條個數
        LineGood = state_bit[0];                                          //確認偵測到磁條
        // if (LCP == 3 || LCP == 6)                                             //偵測到兩條磁條即減速
        // {
        //     speed_down = 1;
        // }
    }
}
