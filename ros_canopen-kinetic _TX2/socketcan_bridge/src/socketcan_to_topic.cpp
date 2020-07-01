/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/string.h>
#include <time.h>
//#include <can_msgs/Frame.h>
#include <can_msgs/Frame.h>
#include <string>
// Add from ino files by Scott
#include "ros.h"
#include "ros/time.h"

//header file for publishing velocities for odom
//header file for cmd_subscribing to "cmd_vel"
//#include "geometry_msgs/Twist.h"
//header files for sub-imu
//#include "geometry_msgs/Vector3.h"
//header file for imu
//#include "delta_base_config.h"
//#include "Motor.h"
//#include "Kinematics.h"
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include "Encoder.h"
//#include <FlexCAN.h>
//#include "delta_msgs/Velocities.h"
#include "delta_msgs/BatteryState.h"
//#include "delta_msgs/MLSState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

/* -----------------------Define global variables----------------------- */

//object for CAN.msg format
can_msgs::Frame msg;
//Establish Battery State Topic
delta_msgs::BatteryState battery_msg;
ros::Publisher battery_pub("battery_state", &battery_msg);
//Declare global variable nh
ros::NodeHandle nh;

//varialbes for BMS
bool isCharging = 0;
int SOC = 0, Temperature = 0, Rack_Voltage = 0, Current = 0;
char Relay_State = 0, System_State = 0;
uint8_t Rack_Warning_State = 0, Rack_Error_State = 0, BMU_Error_State = 0;
bool fully_Charged = 0, Charging = 0, BatteryOVP = 0, Battery_Polarity_Reversed = 0, Battery_Short = 0,
     Fault = 0, Over_Gap = 0, Receiver_OTP = 0, Transmitter_OTP = 0, Wireless_Communication_OK = 0;

namespace can
{
template<> can::FrameFilterSharedPtr tofilter(const XmlRpc::XmlRpcValue  &ct)
{
  XmlRpc::XmlRpcValue t(ct);
  try  // try to read as integer
  {
    uint32_t id = static_cast<int>(t);
    return tofilter(id);
  }
  catch(...)  // else read as string
  {
    return  tofilter(static_cast<std::string>(t));
  }
}
}  // namespace can

namespace socketcan_bridge
{
  
  //void publishBatteryState();
  //void CANReadToStateMsg();

  SocketCANToTopic::SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      can::DriverInterfaceSharedPtr driver)
    {
      //Add global variable for "nh"
      ::nh.advertise(battery_pub);          

      can_topic_ = nh->advertise<can_msgs::Frame>("received_messages",
                                                  nh_param->param("received_messages_queue_size", 10)); 
      driver_ = driver;     
    };

  void SocketCANToTopic::setup()
    {
      
      // register handler for frames and state changes.
      frame_listener_ = driver_->createMsgListenerM(this, &SocketCANToTopic::frameCallback);
      state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
    };

  void SocketCANToTopic::setup(const can::FilteredFrameListener::FilterVector &filters)
  {
    frame_listener_.reset(new can::FilteredFrameListener(driver_,
                                                         std::bind(&SocketCANToTopic::frameCallback,
                                                                   this,
                                                                   std::placeholders::_1),
                                                         filters));

    state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
  }

  void SocketCANToTopic::setup(XmlRpc::XmlRpcValue filters)
  {
      setup(can::tofilters(filters));
  }
  void SocketCANToTopic::setup(ros::NodeHandle nh)
  {
       XmlRpc::XmlRpcValue filters;
       if (nh.getParam("can_ids", filters)) return setup(filters);
       return setup();
  }


  void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
      // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
      if (!f.isValid())
      {
        ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d",
                  f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
        return;
      }
      else
      {
        if (f.is_error)
        {
          // can::tostring cannot be used for dlc > 8 frames. It causes an crash
          // due to usage of boost::array for the data array. The should always work.
          ROS_WARN("Received frame is error: %s", can::tostring(f, true).c_str());
        }
      }

      // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
      convertSocketCANToMessage(f, msg);
 
      msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
      msg.header.stamp = ros::Time::now();
      
      // publishBatteryState(); 
      can_topic_.publish(msg);   
      
      static unsigned long publish_vel_time = 0;
      static unsigned long prev_debug_time = 0;
      static unsigned long prev_search_time = 0;

      CANReadToStateMsg();

    //   if ((clock() - prev_search_time) >= (1000 / 100)) //50
    //   {
    //    publishBatteryState();
    //    publishMLSState();
    //    //stay communication with wireless charger
    //    WriteCANdeviceWirelessCharger();
    //    prev_search_time = clock();
    //   }

    // //this block publishes velocity based on defined rate
    //    if ((clock() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
    //   {
    //    publishVelocities();
    //    publish_vel_time = clock();
    //   }
    // //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    //   // if (DEBUG)
    //   // {
    //   //   if ((clock() - prev_debug_time) >= (1000 / DEBUG_RATE))
    //   //   {
    //   //     printDebug();
    //   //     prev_debug_time = clock();
    //   //   }
    //   // }

      publishBatteryState();
    };


  void SocketCANToTopic::stateCallback(const can::State & s)
    {
      std::string err;
      driver_->translateError(s.internal_error, err);
      if (!s.internal_error)
      {
        ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
      }
      else
      {
        ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
      }
    };


  void SocketCANToTopic::CANReadToStateMsg()
  {
    if (msg.id == 1136)
    {
      SOC = msg.data[0];
      Temperature = msg.data[1];
      Rack_Voltage = (msg.data[2] + (msg.data[3] << 8)) / 1;
      Current = (msg.data[4] + (msg.data[5] << 8)) / 1;      
    }
    if (msg.id == 1137) //0x471 battery II
    {
          Relay_State = msg.data[0];
          Rack_Warning_State = msg.data[1];
          Rack_Error_State = msg.data[2];
          BMU_Error_State = msg.data[3];
          System_State = msg.data[4];
          if (System_State == 7)
              isCharging = 1;
          else
              isCharging = 0;
    }    
  }
  
  void SocketCANToTopic::publishBatteryState()
  {
    battery_msg.SOC = SOC;
    battery_msg.Rack_voltage = Rack_Voltage;
    battery_msg.is_charging = isCharging;
    battery_msg.errors_1 = Rack_Warning_State;
    battery_msg.errors_2 = Rack_Error_State;
    battery_msg.errors_3 = BMU_Error_State;
    battery_pub.publish(&battery_msg);   
  }
  
  void SocketCANToTopic::publishVelocities()
  {
    // //encoder to rpm
    // Kinematics::velocities vel;
    // motor1.updateSpeed(motor1_encoder.read());
    // motor2.updateSpeed(motor2_encoder.read());
    // motor3.updateSpeed(motor3_encoder.read());
    // motor4.updateSpeed(motor4_encoder.read());
    // //calculate the robot's speed based on rpm reading from each motor and platform used.
    // /*
    // if(selftest)
    //     vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());
    // else    vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());  
    // */
    // //rpm to vel
    // vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());
    // //pass velocities to publisher object
    // raw_vel_msg.linear_x = vel.linear_x;
    // raw_vel_msg.linear_y = vel.linear_y;
    // raw_vel_msg.angular_z = vel.angular_z;
    // raw_vel_pub.publish(&raw_vel_msg);
  }

  void SocketCANToTopic::publishMLSState()
  {
    // mls_msg.position = LinePosition;
    // mls_msg.linegood = LineGood;
    // mls_msg.linequantity = LineQuanity;
    // mls_pub.publish(&mls_msg);
  }

};  // namespace socketcan_bridge


