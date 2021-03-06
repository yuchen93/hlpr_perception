// Generated by gencpp from file pc_segmentation/PcFeatures.msg
// DO NOT EDIT!


#ifndef PC_SEGMENTATION_MESSAGE_PCFEATURES_H
#define PC_SEGMENTATION_MESSAGE_PCFEATURES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>


namespace pc_segmentation
{
template <class ContainerAllocator>
struct PcFeatures_
{
  typedef PcFeatures_<ContainerAllocator> Type;

  PcFeatures_()
    : header()
    , transform()
    , points_centroid()
    , points_min()
    , points_max()
    , num_points(0.0)
    , rgba_color()
    , hue(0.0)
    , bb_center()
    , bb_angle(0.0)
    , bb_dims()
    , other_features_size(0)
    , data()  {
    }
  PcFeatures_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , transform(_alloc)
    , points_centroid(_alloc)
    , points_min(_alloc)
    , points_max(_alloc)
    , num_points(0.0)
    , rgba_color(_alloc)
    , hue(0.0)
    , bb_center(_alloc)
    , bb_angle(0.0)
    , bb_dims(_alloc)
    , other_features_size(0)
    , data(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _transform_type;
  _transform_type transform;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _points_centroid_type;
  _points_centroid_type points_centroid;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _points_min_type;
  _points_min_type points_min;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _points_max_type;
  _points_max_type points_max;

   typedef double _num_points_type;
  _num_points_type num_points;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _rgba_color_type;
  _rgba_color_type rgba_color;

   typedef double _hue_type;
  _hue_type hue;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _bb_center_type;
  _bb_center_type bb_center;

   typedef double _bb_angle_type;
  _bb_angle_type bb_angle;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _bb_dims_type;
  _bb_dims_type bb_dims;

   typedef uint32_t _other_features_size_type;
  _other_features_size_type other_features_size;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::pc_segmentation::PcFeatures_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pc_segmentation::PcFeatures_<ContainerAllocator> const> ConstPtr;

}; // struct PcFeatures_

typedef ::pc_segmentation::PcFeatures_<std::allocator<void> > PcFeatures;

typedef boost::shared_ptr< ::pc_segmentation::PcFeatures > PcFeaturesPtr;
typedef boost::shared_ptr< ::pc_segmentation::PcFeatures const> PcFeaturesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pc_segmentation::PcFeatures_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pc_segmentation::PcFeatures_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pc_segmentation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'pc_segmentation': ['/home/baris/baris_ws/src/pc_segmentation/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pc_segmentation::PcFeatures_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pc_segmentation::PcFeatures_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pc_segmentation::PcFeatures_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "623fac7b3275a81643e666d8b500f34f";
  }

  static const char* value(const ::pc_segmentation::PcFeatures_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x623fac7b3275a816ULL;
  static const uint64_t static_value2 = 0x43e666d8b500f34fULL;
};

template<class ContainerAllocator>
struct DataType< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pc_segmentation/PcFeatures";
  }

  static const char* value(const ::pc_segmentation::PcFeatures_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# PcFeatures for a single cluster\n\
\n\
Header header\n\
\n\
# Object transform, however calculated\n\
geometry_msgs/Transform transform\n\
\n\
\n\
#################################\n\
# Raw point Related\n\
\n\
# Cluster centroid, min, max and number of points\n\
geometry_msgs/Vector3 points_centroid\n\
geometry_msgs/Vector3 points_min   #<pc min x, pc min y, pc min z>\n\
geometry_msgs/Vector3 points_max   #<pc max x, pc max y, pc max z>\n\
float64 num_points\n\
\n\
# Average color (RGBA nad hue)\n\
std_msgs/ColorRGBA rgba_color\n\
float64 hue\n\
\n\
#################################\n\
#Bounding box\n\
\n\
#position wrt sensor and angle wrt table normal\n\
geometry_msgs/Vector3 bb_center\n\
float64 bb_angle\n\
\n\
# Bounding box dimensions\n\
geometry_msgs/Vector3 bb_dims\n\
\n\
#################################\n\
#Other such as vfh, color hist etc. Unpacking will be on the other side\n\
\n\
uint32 other_features_size\n\
float64[] data\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: std_msgs/ColorRGBA\n\
float32 r\n\
float32 g\n\
float32 b\n\
float32 a\n\
";
  }

  static const char* value(const ::pc_segmentation::PcFeatures_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.transform);
      stream.next(m.points_centroid);
      stream.next(m.points_min);
      stream.next(m.points_max);
      stream.next(m.num_points);
      stream.next(m.rgba_color);
      stream.next(m.hue);
      stream.next(m.bb_center);
      stream.next(m.bb_angle);
      stream.next(m.bb_dims);
      stream.next(m.other_features_size);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct PcFeatures_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pc_segmentation::PcFeatures_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pc_segmentation::PcFeatures_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "transform: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.transform);
    s << indent << "points_centroid: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.points_centroid);
    s << indent << "points_min: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.points_min);
    s << indent << "points_max: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.points_max);
    s << indent << "num_points: ";
    Printer<double>::stream(s, indent + "  ", v.num_points);
    s << indent << "rgba_color: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.rgba_color);
    s << indent << "hue: ";
    Printer<double>::stream(s, indent + "  ", v.hue);
    s << indent << "bb_center: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.bb_center);
    s << indent << "bb_angle: ";
    Printer<double>::stream(s, indent + "  ", v.bb_angle);
    s << indent << "bb_dims: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.bb_dims);
    s << indent << "other_features_size: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.other_features_size);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PC_SEGMENTATION_MESSAGE_PCFEATURES_H
