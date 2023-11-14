#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

visualization_msgs::msg::Marker make_bbox_msg(rclcpp::Node* node, std::string frame_id, std::string ns, 
        float px, float py, float pz, float sx, float sy, float sz);

visualization_msgs::msg::Marker make_bbox_msg(rclcpp::Node* node, std::string frame_id, std::string ns, 
        std::vector<float> bounds);

void to_tf(geometry_msgs::msg::Transform& in, tf2::Transform& out);

void to_tfmsg(tf2::Transform& in, geometry_msgs::msg::Transform& out);

void tf2pose(tf2::Transform& in, geometry_msgs::msg::Pose& out);

Eigen::Isometry3d interpolateIsometry3d(const Eigen::Isometry3d& start,
                                       const Eigen::Isometry3d& end,
                                       double t);



template <class T>
T get_safe_param(std::shared_ptr<rclcpp::Node> n, const std::string name, T def)
{
  if (n->has_parameter(name)) {
    T p(def);
    n->get_parameter(name, p);
    return (p);
  }
  return (n->declare_parameter<T>(name, def));
} 