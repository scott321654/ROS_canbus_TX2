#ifndef _ROS_delta_msgs_DigitalIO_h
#define _ROS_delta_msgs_DigitalIO_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace delta_msgs
{

  class DigitalIO : public ros::Msg
  {
    public:
      bool values[16];

    DigitalIO():
      values()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 16; i++){
      union {
        bool real;
        uint8_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 16; i++){
      union {
        bool real;
        uint8_t base;
      } u_valuesi;
      u_valuesi.base = 0;
      u_valuesi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->values[i] = u_valuesi.real;
      offset += sizeof(this->values[i]);
      }
     return offset;
    }

    const char * getType(){ return "delta_msgs/DigitalIO"; };
    const char * getMD5(){ return "842dc18b08ce52a52b321bb3e51a8d68"; };

  };

}
#endif
