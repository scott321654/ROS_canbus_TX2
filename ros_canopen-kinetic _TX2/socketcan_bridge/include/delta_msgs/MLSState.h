#ifndef _ROS_delta_msgs_MLSState_h
#define _ROS_delta_msgs_MLSState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace delta_msgs
{

  class MLSState : public ros::Msg
  {
    public:
      typedef int16_t _position_type;
      _position_type position;
      typedef bool _linegood_type;
      _linegood_type linegood;
      typedef int8_t _linequantity_type;
      _linequantity_type linequantity;

    MLSState():
      position(0),
      linegood(0),
      linequantity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position);
      union {
        bool real;
        uint8_t base;
      } u_linegood;
      u_linegood.real = this->linegood;
      *(outbuffer + offset + 0) = (u_linegood.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->linegood);
      union {
        int8_t real;
        uint8_t base;
      } u_linequantity;
      u_linequantity.real = this->linequantity;
      *(outbuffer + offset + 0) = (u_linequantity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->linequantity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        bool real;
        uint8_t base;
      } u_linegood;
      u_linegood.base = 0;
      u_linegood.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->linegood = u_linegood.real;
      offset += sizeof(this->linegood);
      union {
        int8_t real;
        uint8_t base;
      } u_linequantity;
      u_linequantity.base = 0;
      u_linequantity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->linequantity = u_linequantity.real;
      offset += sizeof(this->linequantity);
     return offset;
    }

    const char * getType(){ return "delta_msgs/MLSState"; };
    const char * getMD5(){ return "4f746a1d8533bea0514d2768664dadee"; };

  };

}
#endif
