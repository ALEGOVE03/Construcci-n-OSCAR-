// Generated by gencpp from file arduino_Thor/sensors_raw.msg
// DO NOT EDIT!


#ifndef ARDUINO_THOR_MESSAGE_SENSORS_RAW_H
#define ARDUINO_THOR_MESSAGE_SENSORS_RAW_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace arduino_Thor
{
template <class ContainerAllocator>
struct sensors_raw_
{
  typedef sensors_raw_<ContainerAllocator> Type;

  sensors_raw_()
    : joint_1(0)
    , joint_2(0)
    , joint_3(0)
    , joint_4(0)
    , joint_5(0)
    , joint_6(0)
    , joint_5post(0)
    , joint_6post(0)
    , gripperPos(0)  {
    }
  sensors_raw_(const ContainerAllocator& _alloc)
    : joint_1(0)
    , joint_2(0)
    , joint_3(0)
    , joint_4(0)
    , joint_5(0)
    , joint_6(0)
    , joint_5post(0)
    , joint_6post(0)
    , gripperPos(0)  {
  (void)_alloc;
    }



   typedef int16_t _joint_1_type;
  _joint_1_type joint_1;

   typedef int16_t _joint_2_type;
  _joint_2_type joint_2;

   typedef int16_t _joint_3_type;
  _joint_3_type joint_3;

   typedef int16_t _joint_4_type;
  _joint_4_type joint_4;

   typedef int16_t _joint_5_type;
  _joint_5_type joint_5;

   typedef int16_t _joint_6_type;
  _joint_6_type joint_6;

   typedef int16_t _joint_5post_type;
  _joint_5post_type joint_5post;

   typedef int16_t _joint_6post_type;
  _joint_6post_type joint_6post;

   typedef int16_t _gripperPos_type;
  _gripperPos_type gripperPos;





  typedef boost::shared_ptr< ::arduino_Thor::sensors_raw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arduino_Thor::sensors_raw_<ContainerAllocator> const> ConstPtr;

}; // struct sensors_raw_

typedef ::arduino_Thor::sensors_raw_<std::allocator<void> > sensors_raw;

typedef boost::shared_ptr< ::arduino_Thor::sensors_raw > sensors_rawPtr;
typedef boost::shared_ptr< ::arduino_Thor::sensors_raw const> sensors_rawConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arduino_Thor::sensors_raw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arduino_Thor::sensors_raw_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::arduino_Thor::sensors_raw_<ContainerAllocator1> & lhs, const ::arduino_Thor::sensors_raw_<ContainerAllocator2> & rhs)
{
  return lhs.joint_1 == rhs.joint_1 &&
    lhs.joint_2 == rhs.joint_2 &&
    lhs.joint_3 == rhs.joint_3 &&
    lhs.joint_4 == rhs.joint_4 &&
    lhs.joint_5 == rhs.joint_5 &&
    lhs.joint_6 == rhs.joint_6 &&
    lhs.joint_5post == rhs.joint_5post &&
    lhs.joint_6post == rhs.joint_6post &&
    lhs.gripperPos == rhs.gripperPos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::arduino_Thor::sensors_raw_<ContainerAllocator1> & lhs, const ::arduino_Thor::sensors_raw_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace arduino_Thor

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arduino_Thor::sensors_raw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arduino_Thor::sensors_raw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arduino_Thor::sensors_raw_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9f3018bca4546b3bc893ddcc513190e3";
  }

  static const char* value(const ::arduino_Thor::sensors_raw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9f3018bca4546b3bULL;
  static const uint64_t static_value2 = 0xc893ddcc513190e3ULL;
};

template<class ContainerAllocator>
struct DataType< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arduino_Thor/sensors_raw";
  }

  static const char* value(const ::arduino_Thor::sensors_raw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 joint_1\n"
"int16 joint_2\n"
"int16 joint_3\n"
"int16 joint_4\n"
"int16 joint_5\n"
"int16 joint_6\n"
"int16 joint_5post\n"
"int16 joint_6post\n"
"int16 gripperPos\n"
;
  }

  static const char* value(const ::arduino_Thor::sensors_raw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_1);
      stream.next(m.joint_2);
      stream.next(m.joint_3);
      stream.next(m.joint_4);
      stream.next(m.joint_5);
      stream.next(m.joint_6);
      stream.next(m.joint_5post);
      stream.next(m.joint_6post);
      stream.next(m.gripperPos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sensors_raw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arduino_Thor::sensors_raw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arduino_Thor::sensors_raw_<ContainerAllocator>& v)
  {
    s << indent << "joint_1: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_1);
    s << indent << "joint_2: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_2);
    s << indent << "joint_3: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_3);
    s << indent << "joint_4: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_4);
    s << indent << "joint_5: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_5);
    s << indent << "joint_6: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_6);
    s << indent << "joint_5post: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_5post);
    s << indent << "joint_6post: ";
    Printer<int16_t>::stream(s, indent + "  ", v.joint_6post);
    s << indent << "gripperPos: ";
    Printer<int16_t>::stream(s, indent + "  ", v.gripperPos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDUINO_THOR_MESSAGE_SENSORS_RAW_H
