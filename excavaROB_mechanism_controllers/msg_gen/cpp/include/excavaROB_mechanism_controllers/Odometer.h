/* Auto-generated by genmsg_cpp for file /home/cmastalli/ros_workspace/excavabot/excavaROB_mechanism_controllers/msg/Odometer.msg */
#ifndef EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_ODOMETER_H
#define EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_ODOMETER_H
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


namespace excavaROB_mechanism_controllers
{
template <class ContainerAllocator>
struct Odometer_ {
  typedef Odometer_<ContainerAllocator> Type;

  Odometer_()
  : distance(0.0)
  , angle(0.0)
  {
  }

  Odometer_(const ContainerAllocator& _alloc)
  : distance(0.0)
  , angle(0.0)
  {
  }

  typedef double _distance_type;
  double distance;

  typedef double _angle_type;
  double angle;


  typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Odometer
typedef  ::excavaROB_mechanism_controllers::Odometer_<std::allocator<void> > Odometer;

typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::Odometer> OdometerPtr;
typedef boost::shared_ptr< ::excavaROB_mechanism_controllers::Odometer const> OdometerConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace excavaROB_mechanism_controllers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1f1d53743f4592ee455aa3eaf9019457";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1f1d53743f4592eeULL;
  static const uint64_t static_value2 = 0x455aa3eaf9019457ULL;
};

template<class ContainerAllocator>
struct DataType< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "excavaROB_mechanism_controllers/Odometer";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 distance #total distance traveled (meters)\n\
float64 angle #total angle traveled (radians)\n\
";
  }

  static const char* value(const  ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.distance);
    stream.next(m.angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Odometer_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::excavaROB_mechanism_controllers::Odometer_<ContainerAllocator> & v) 
  {
    s << indent << "distance: ";
    Printer<double>::stream(s, indent + "  ", v.distance);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EXCAVAROB_MECHANISM_CONTROLLERS_MESSAGE_ODOMETER_H

