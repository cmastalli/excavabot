/* Auto-generated by genmsg_cpp for file /home/cmastalli/ros_workspace/excavabot/excavaROB_mechanism_controllers/msg/BaseControllerState.msg */
#ifndef EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_BASECONTROLLERSTATE_H
#define EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_BASECONTROLLERSTATE_H
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
#include "excavaROB_msgs/VelocityBase.h"
#include "excavaROB_msgs/VelocityBase.h"
#include "excavaROB_msgs/VelocityBase.h"

namespace excavaROB_mechanism_controllers
{
template <class ContainerAllocator>
struct BaseControllerState_ {
  typedef BaseControllerState_<ContainerAllocator> Type;

  BaseControllerState_()
  : header()
  , joint_names()
  , velocity_measured()
  , velocity_desired()
  , velocity_error()
  , control_signal_right_wheel(0.0)
  , control_signal_left_wheel(0.0)
  {
  }

  BaseControllerState_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , joint_names(_alloc)
  , velocity_measured(_alloc)
  , velocity_desired(_alloc)
  , velocity_error(_alloc)
  , control_signal_right_wheel(0.0)
  , control_signal_left_wheel(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef  ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  _velocity_measured_type;
   ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  velocity_measured;

  typedef  ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  _velocity_desired_type;
   ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  velocity_desired;

  typedef  ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  _velocity_error_type;
   ::excavaROB_msgs::VelocityBase_<ContainerAllocator>  velocity_error;

  typedef double _control_signal_right_wheel_type;
  double control_signal_right_wheel;

  typedef double _control_signal_left_wheel_type;
  double control_signal_left_wheel;


  typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BaseControllerState
typedef  ::excavaROB_mechanism_controllers::BaseControllerState_<std::allocator<void> > BaseControllerState;

typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::BaseControllerState> BaseControllerStatePtr;
typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::BaseControllerState const> BaseControllerStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace excavaROB_mechanism_controllers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0842f12a4344161afabd24de08a03a26";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0842f12a4344161aULL;
  static const uint64_t static_value2 = 0xfabd24de08a03a26ULL;
};

template<class ContainerAllocator>
struct DataType< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "excavaROB_mechanism_controllers/BaseControllerState";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# This represents an base controller state with reference coordinate frame and timestamp.\n\
Header header\n\
# Wheel joints \n\
string[] joint_names\n\
# Linear and angular measured, desired and error base velocities\n\
excavaROB_msgs/VelocityBase velocity_measured\n\
excavaROB_msgs/VelocityBase velocity_desired\n\
excavaROB_msgs/VelocityBase velocity_error\n\
# Control signals\n\
float64 control_signal_right_wheel\n\
float64 control_signal_left_wheel\n\
\n\
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
MSG: excavaROB_msgs/VelocityBase\n\
# This contains the linear and angular base velocity\n\
float64 linear\n\
float64 angular\n\
\n\
";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.joint_names);
    stream.next(m.velocity_measured);
    stream.next(m.velocity_desired);
    stream.next(m.velocity_error);
    stream.next(m.control_signal_right_wheel);
    stream.next(m.control_signal_left_wheel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BaseControllerState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::excavaROB_mechanism_controllers::BaseControllerState_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "velocity_measured: ";
s << std::endl;
    Printer< ::excavaROB_msgs::VelocityBase_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_measured);
    s << indent << "velocity_desired: ";
s << std::endl;
    Printer< ::excavaROB_msgs::VelocityBase_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_desired);
    s << indent << "velocity_error: ";
s << std::endl;
    Printer< ::excavaROB_msgs::VelocityBase_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_error);
    s << indent << "control_signal_right_wheel: ";
    Printer<double>::stream(s, indent + "  ", v.control_signal_right_wheel);
    s << indent << "control_signal_left_wheel: ";
    Printer<double>::stream(s, indent + "  ", v.control_signal_left_wheel);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_BASECONTROLLERSTATE_H

