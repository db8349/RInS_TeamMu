// Generated by gencpp from file exercise1/ReverseRequest.msg
// DO NOT EDIT!


#ifndef EXERCISE1_MESSAGE_REVERSEREQUEST_H
#define EXERCISE1_MESSAGE_REVERSEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace exercise1
{
template <class ContainerAllocator>
struct ReverseRequest_
{
  typedef ReverseRequest_<ContainerAllocator> Type;

  ReverseRequest_()
    : content()  {
    }
  ReverseRequest_(const ContainerAllocator& _alloc)
    : content(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _content_type;
  _content_type content;





  typedef boost::shared_ptr< ::exercise1::ReverseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::exercise1::ReverseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ReverseRequest_

typedef ::exercise1::ReverseRequest_<std::allocator<void> > ReverseRequest;

typedef boost::shared_ptr< ::exercise1::ReverseRequest > ReverseRequestPtr;
typedef boost::shared_ptr< ::exercise1::ReverseRequest const> ReverseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::exercise1::ReverseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::exercise1::ReverseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace exercise1

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'exercise1': ['/home/team_mu/ROS/src/exercise1/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::exercise1::ReverseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exercise1::ReverseRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exercise1::ReverseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exercise1::ReverseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exercise1::ReverseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exercise1::ReverseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::exercise1::ReverseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c2e84951ee6d0addf437bfddd5b19734";
  }

  static const char* value(const ::exercise1::ReverseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc2e84951ee6d0addULL;
  static const uint64_t static_value2 = 0xf437bfddd5b19734ULL;
};

template<class ContainerAllocator>
struct DataType< ::exercise1::ReverseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "exercise1/ReverseRequest";
  }

  static const char* value(const ::exercise1::ReverseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::exercise1::ReverseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
string content\n\
\n\
\n\
";
  }

  static const char* value(const ::exercise1::ReverseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::exercise1::ReverseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.content);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReverseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::exercise1::ReverseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::exercise1::ReverseRequest_<ContainerAllocator>& v)
  {
    s << indent << "content: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.content);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXERCISE1_MESSAGE_REVERSEREQUEST_H
