#ifndef _ROS_object_msgs_Object_h
#define _ROS_object_msgs_Object_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "object_recognition_msgs/ObjectType.h"
#include "shape_msgs/SolidPrimitive.h"
#include "geometry_msgs/Pose.h"
#include "shape_msgs/Mesh.h"
#include "shape_msgs/Plane.h"

namespace object_msgs
{

  class Object : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* name;
      object_recognition_msgs::ObjectType type;
      uint8_t primitives_length;
      shape_msgs::SolidPrimitive st_primitives;
      shape_msgs::SolidPrimitive * primitives;
      uint8_t primitive_poses_length;
      geometry_msgs::Pose st_primitive_poses;
      geometry_msgs::Pose * primitive_poses;
      uint8_t meshes_length;
      shape_msgs::Mesh st_meshes;
      shape_msgs::Mesh * meshes;
      uint8_t mesh_poses_length;
      geometry_msgs::Pose st_mesh_poses;
      geometry_msgs::Pose * mesh_poses;
      uint8_t planes_length;
      shape_msgs::Plane st_planes;
      shape_msgs::Plane * planes;
      uint8_t plane_poses_length;
      geometry_msgs::Pose st_plane_poses;
      geometry_msgs::Pose * plane_poses;
      int32_t primitive_origin;
      int32_t mesh_origin;
      geometry_msgs::Pose origin;
      int8_t content;
      enum { SHAPE = 0 };
      enum { POSE = 1 };
      enum { ORIGIN_AVERAGE = -1 };
      enum { ORIGIN_UNDEFINED = -2 };
      enum { ORIGIN_CUSTOM = -3 };

    Object():
      header(),
      name(""),
      type(),
      primitives_length(0), primitives(NULL),
      primitive_poses_length(0), primitive_poses(NULL),
      meshes_length(0), meshes(NULL),
      mesh_poses_length(0), mesh_poses(NULL),
      planes_length(0), planes(NULL),
      plane_poses_length(0), plane_poses(NULL),
      primitive_origin(0),
      mesh_origin(0),
      origin(),
      content(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->type.serialize(outbuffer + offset);
      *(outbuffer + offset++) = primitives_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < primitives_length; i++){
      offset += this->primitives[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = primitive_poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < primitive_poses_length; i++){
      offset += this->primitive_poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = meshes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < meshes_length; i++){
      offset += this->meshes[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = mesh_poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < mesh_poses_length; i++){
      offset += this->mesh_poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = planes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < planes_length; i++){
      offset += this->planes[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = plane_poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < plane_poses_length; i++){
      offset += this->plane_poses[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_primitive_origin;
      u_primitive_origin.real = this->primitive_origin;
      *(outbuffer + offset + 0) = (u_primitive_origin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_primitive_origin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_primitive_origin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_primitive_origin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->primitive_origin);
      union {
        int32_t real;
        uint32_t base;
      } u_mesh_origin;
      u_mesh_origin.real = this->mesh_origin;
      *(outbuffer + offset + 0) = (u_mesh_origin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mesh_origin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mesh_origin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mesh_origin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mesh_origin);
      offset += this->origin.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_content;
      u_content.real = this->content;
      *(outbuffer + offset + 0) = (u_content.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->content);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->type.deserialize(inbuffer + offset);
      uint8_t primitives_lengthT = *(inbuffer + offset++);
      if(primitives_lengthT > primitives_length)
        this->primitives = (shape_msgs::SolidPrimitive*)realloc(this->primitives, primitives_lengthT * sizeof(shape_msgs::SolidPrimitive));
      offset += 3;
      primitives_length = primitives_lengthT;
      for( uint8_t i = 0; i < primitives_length; i++){
      offset += this->st_primitives.deserialize(inbuffer + offset);
        memcpy( &(this->primitives[i]), &(this->st_primitives), sizeof(shape_msgs::SolidPrimitive));
      }
      uint8_t primitive_poses_lengthT = *(inbuffer + offset++);
      if(primitive_poses_lengthT > primitive_poses_length)
        this->primitive_poses = (geometry_msgs::Pose*)realloc(this->primitive_poses, primitive_poses_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      primitive_poses_length = primitive_poses_lengthT;
      for( uint8_t i = 0; i < primitive_poses_length; i++){
      offset += this->st_primitive_poses.deserialize(inbuffer + offset);
        memcpy( &(this->primitive_poses[i]), &(this->st_primitive_poses), sizeof(geometry_msgs::Pose));
      }
      uint8_t meshes_lengthT = *(inbuffer + offset++);
      if(meshes_lengthT > meshes_length)
        this->meshes = (shape_msgs::Mesh*)realloc(this->meshes, meshes_lengthT * sizeof(shape_msgs::Mesh));
      offset += 3;
      meshes_length = meshes_lengthT;
      for( uint8_t i = 0; i < meshes_length; i++){
      offset += this->st_meshes.deserialize(inbuffer + offset);
        memcpy( &(this->meshes[i]), &(this->st_meshes), sizeof(shape_msgs::Mesh));
      }
      uint8_t mesh_poses_lengthT = *(inbuffer + offset++);
      if(mesh_poses_lengthT > mesh_poses_length)
        this->mesh_poses = (geometry_msgs::Pose*)realloc(this->mesh_poses, mesh_poses_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      mesh_poses_length = mesh_poses_lengthT;
      for( uint8_t i = 0; i < mesh_poses_length; i++){
      offset += this->st_mesh_poses.deserialize(inbuffer + offset);
        memcpy( &(this->mesh_poses[i]), &(this->st_mesh_poses), sizeof(geometry_msgs::Pose));
      }
      uint8_t planes_lengthT = *(inbuffer + offset++);
      if(planes_lengthT > planes_length)
        this->planes = (shape_msgs::Plane*)realloc(this->planes, planes_lengthT * sizeof(shape_msgs::Plane));
      offset += 3;
      planes_length = planes_lengthT;
      for( uint8_t i = 0; i < planes_length; i++){
      offset += this->st_planes.deserialize(inbuffer + offset);
        memcpy( &(this->planes[i]), &(this->st_planes), sizeof(shape_msgs::Plane));
      }
      uint8_t plane_poses_lengthT = *(inbuffer + offset++);
      if(plane_poses_lengthT > plane_poses_length)
        this->plane_poses = (geometry_msgs::Pose*)realloc(this->plane_poses, plane_poses_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      plane_poses_length = plane_poses_lengthT;
      for( uint8_t i = 0; i < plane_poses_length; i++){
      offset += this->st_plane_poses.deserialize(inbuffer + offset);
        memcpy( &(this->plane_poses[i]), &(this->st_plane_poses), sizeof(geometry_msgs::Pose));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_primitive_origin;
      u_primitive_origin.base = 0;
      u_primitive_origin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_primitive_origin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_primitive_origin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_primitive_origin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->primitive_origin = u_primitive_origin.real;
      offset += sizeof(this->primitive_origin);
      union {
        int32_t real;
        uint32_t base;
      } u_mesh_origin;
      u_mesh_origin.base = 0;
      u_mesh_origin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mesh_origin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mesh_origin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mesh_origin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mesh_origin = u_mesh_origin.real;
      offset += sizeof(this->mesh_origin);
      offset += this->origin.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_content;
      u_content.base = 0;
      u_content.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->content = u_content.real;
      offset += sizeof(this->content);
     return offset;
    }

    const char * getType(){ return "object_msgs/Object"; };
    const char * getMD5(){ return "59a4dbed61a3869d90b7226cdd261082"; };

  };

}
#endif