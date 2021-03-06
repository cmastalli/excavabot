/* Auto-generated by genmsg_cpp for file /home/cmastalli/ros_workspace/excavabot/excavaROB_common/excavaROB_msgs/msg/PoseStamped.msg */
#ifndef EXCAVAROB_MSGS_MESSAGE_POSESTAMPED_H
#define EXCAVAROB_MSGS_MESSAGE_POSESTAMPED_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "excavaROB_msgs/Pose.h"

namespace excavaROB_msgs
{
template <class ContainerAllocator>
struct PoseStamped_ {
  typedef PoseStamped_<ContainerAllocator> Type;

  PoseStamped_()
  : header()
  , pose()
  {
  }

  PoseStamped_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , pose(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::excavaROB_msgs::Pose_<ContainerAllocator>  _pose_type;
   ::excavaROB_msgs::Pose_<ContainerAllocator>  pose;


  typedef boost::shared_ptr< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::excavaROB_msgs::PoseStamped_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PoseStamped
typedef  ::excavaROB_msgs::PoseStamped_<std::allocator<void> > PoseStamped;

typedef boost::shared_ptr< ::excavaROB_msgs::PoseStamped> PoseStampedPtr;
typedef boost::shared_ptr< ::excavaROB_msgs::PoseStamped const> PoseStampedConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::excavaROB_msgs::PoseStamped_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace excavaROB_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::excavaROB_msgs::PoseStamped_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bdbc5283b3f3d17b974ff7a29bdb3a7a";
  }

  static const char* value(const  ::excavaROB_msgs::PoseStamped_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbdbc5283b3f3d17bULL;
  static const uint64_t static_value2 = 0x974ff7a29bdb3a7aULL;
};

template<class ContainerAllocator>
struct DataType< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "excavaROB_msgs/PoseStamped";
  }

  static const char* value(const  ::excavaROB_msgs::PoseStamped_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: excavaROB_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation (pitch angle of the bucket). \n\
geometry_msgs/Point position\n\
float64 pitch\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
";
  }

  static const char* value(const  ::excavaROB_msgs::PoseStamped_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::excavaROB_msgs::PoseStamped_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.pose);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PoseStamped_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::excavaROB_msgs::PoseStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::excavaROB_msgs::PoseStamped_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
s << std::endl;
    Printer< ::excavaROB_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EXCAVAROB_MSGS_MESSAGE_POSESTAMPED_H

