#ifndef _ROS_delta_msgs_MagneticGuide_h
#define _ROS_delta_msgs_MagneticGuide_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace delta_msgs
{

  class MagneticGuide : public ros::Msg
  {
    public:
      typedef int8_t _control_type;
      _control_type control;
      enum { STOP =  0 };
      enum { FORWARD =  1 };
      enum { BACKWARD =  2 };

    MagneticGuide():
      control(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_control;
      u_control.real = this->control;
      *(outbuffer + offset + 0) = (u_control.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_control;
      u_control.base = 0;
      u_control.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->control = u_control.real;
      offset += sizeof(this->control);
     return offset;
    }

    const char * getType(){ return "delta_msgs/MagneticGuide"; };
    const char * getMD5(){ return "1c524e970de528c5cd36b0a5b263ddf3"; };

  };

}
#endif
