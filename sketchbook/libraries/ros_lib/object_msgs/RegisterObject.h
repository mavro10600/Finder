#ifndef _ROS_SERVICE_RegisterObject_h
#define _ROS_SERVICE_RegisterObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace object_msgs
{

static const char REGISTEROBJECT[] = "object_msgs/RegisterObject";

  class RegisterObjectRequest : public ros::Msg
  {
    public:
      const char* name;

    RegisterObjectRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return REGISTEROBJECT; };
    const char * getMD5(){ return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

  class RegisterObjectResponse : public ros::Msg
  {
    public:
      int8_t success;
      enum { SUCCESS = 0 };
      enum { EXISTS = 1 };
      enum { ERROR_INFO = 2 };

    RegisterObjectResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return REGISTEROBJECT; };
    const char * getMD5(){ return "b9c25147bff223b8f5fb48665bb7c161"; };

  };

  class RegisterObject {
    public:
    typedef RegisterObjectRequest Request;
    typedef RegisterObjectResponse Response;
  };

}
#endif
