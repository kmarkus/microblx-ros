#ifndef ROS_KDL_CPP
#define ROS_KDL_CPP

#include <kdl_conversions/kdl_msg.h>
#include "ros_kdl.hpp"

// this actually defines the read/write helpers
#include <ubxkdl.hpp>

// define overloaded conversion functions
void convert(const geometry_msgs::Point &m, KDL::Vector &k) { tf::pointMsgToKDL(m,k); }
void convert(const KDL::Vector &k, geometry_msgs::Point &m) { tf::pointKDLToMsg(k,m); }
void convert(const geometry_msgs::Pose &m, KDL::Frame &k) { tf::poseMsgToKDL(m,k); }
void convert(const KDL::Frame &k, geometry_msgs::Pose &m) { tf::poseKDLToMsg(k,m); }
void convert(const geometry_msgs::Quaternion &m, KDL::Rotation &k) { tf::quaternionMsgToKDL(m,k); }
void convert(const KDL::Rotation &k, geometry_msgs::Quaternion &m) { tf::quaternionKDLToMsg(k,m); }
void convert(const geometry_msgs::Transform &m, KDL::Frame &k) { tf::transformMsgToKDL(m,k); }
void convert(const KDL::Frame &k, geometry_msgs::Transform &m) { tf::transformKDLToMsg(k,m); }
void convert(const geometry_msgs::Twist &m, KDL::Twist &k) { tf::twistMsgToKDL(m,k); }
void convert(const KDL::Twist &k, geometry_msgs::Twist &m) { tf::twistKDLToMsg(k,m); }
void convert(const geometry_msgs::Vector3 &m, KDL::Vector &k) { tf::vectorMsgToKDL(m,k); }
void convert(const KDL::Vector &k, geometry_msgs::Vector3 &m) { tf::vectorKDLToMsg(k,m); }
void convert(const geometry_msgs::Wrench &m, KDL::Wrench &k) { tf::wrenchMsgToKDL(m,k); }
void convert(const KDL::Wrench &k, geometry_msgs::Wrench &m) { tf::wrenchKDLToMsg(k,m); }

#endif
