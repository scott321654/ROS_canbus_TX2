#ifndef _ROS_delta_msgs_BatteryState_h
#define _ROS_delta_msgs_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace delta_msgs
{

  class BatteryState : public ros::Msg
  {
    public:
      typedef float _SOC_type;
      _SOC_type SOC;
      typedef float _Rack_voltage_type;
      _Rack_voltage_type Rack_voltage;
      typedef bool _is_charging_type;
      _is_charging_type is_charging;
      typedef uint8_t _errors_1_type;
      _errors_1_type errors_1;
      typedef uint8_t _errors_2_type;
      _errors_2_type errors_2;
      typedef uint8_t _errors_3_type;
      _errors_3_type errors_3;

    BatteryState():
      SOC(0),
      Rack_voltage(0),
      is_charging(0),
      errors_1(0),
      errors_2(0),
      errors_3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_SOC;
      u_SOC.real = this->SOC;
      *(outbuffer + offset + 0) = (u_SOC.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SOC.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SOC.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SOC.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->SOC);
      union {
        float real;
        uint32_t base;
      } u_Rack_voltage;
      u_Rack_voltage.real = this->Rack_voltage;
      *(outbuffer + offset + 0) = (u_Rack_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Rack_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Rack_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Rack_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Rack_voltage);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.real = this->is_charging;
      *(outbuffer + offset + 0) = (u_is_charging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_charging);
      *(outbuffer + offset + 0) = (this->errors_1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errors_1);
      *(outbuffer + offset + 0) = (this->errors_2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errors_2);
      *(outbuffer + offset + 0) = (this->errors_3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errors_3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_SOC;
      u_SOC.base = 0;
      u_SOC.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SOC.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SOC.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SOC.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->SOC = u_SOC.real;
      offset += sizeof(this->SOC);
      union {
        float real;
        uint32_t base;
      } u_Rack_voltage;
      u_Rack_voltage.base = 0;
      u_Rack_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Rack_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Rack_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Rack_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Rack_voltage = u_Rack_voltage.real;
      offset += sizeof(this->Rack_voltage);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.base = 0;
      u_is_charging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_charging = u_is_charging.real;
      offset += sizeof(this->is_charging);
      this->errors_1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->errors_1);
      this->errors_2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->errors_2);
      this->errors_3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->errors_3);
     return offset;
    }

    const char * getType(){ return "delta_msgs/BatteryState"; };
    const char * getMD5(){ return "dd1ff163d8c15639beab4588274a262a"; };

  };

}
#endif
