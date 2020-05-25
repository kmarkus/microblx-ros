#include <ubx/ubx.h>
#include <ros/ros.h>
#include "types/ubxros_conn.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <kdl/frames.hpp>

// overloaded conversion functions
void convert(const geometry_msgs::Point &m, KDL::Vector &k);
void convert(const KDL::Vector &k, geometry_msgs::Point &m);
void convert(const geometry_msgs::Pose &m, KDL::Frame &k);
void convert(const KDL::Frame &k, geometry_msgs::Pose &m);
void convert(const geometry_msgs::Quaternion &m, KDL::Rotation &k);
void convert(const KDL::Rotation &k, geometry_msgs::Quaternion &m);
void convert(const geometry_msgs::Transform &m, KDL::Frame &k);
void convert(const KDL::Frame &k, geometry_msgs::Transform &m);
void convert(const geometry_msgs::Twist &m, KDL::Twist &k);
void convert(const KDL::Twist &k, geometry_msgs::Twist &m);
void convert(const geometry_msgs::Vector3 &m, KDL::Vector &k);
void convert(const KDL::Vector &k, geometry_msgs::Vector3 &m);
void convert(const geometry_msgs::Wrench &m, KDL::Wrench &k);
void convert(const KDL::Wrench &k, geometry_msgs::Wrench &m);

long portWrite(const ubx_port_t*, const KDL::Vector*, int);
long portWrite(const ubx_port_t*, const KDL::Rotation*, int);
long portWrite(const ubx_port_t*, const KDL::Frame*, int);
long portWrite(const ubx_port_t*, const KDL::Twist*, int);
long portWrite(const ubx_port_t*, const KDL::Wrench*, int);

long portRead(const ubx_port_t*, KDL::Vector*, int);
long portRead(const ubx_port_t*, KDL::Rotation*, int);
long portRead(const ubx_port_t*, KDL::Frame*, int);
long portRead(const ubx_port_t*, KDL::Twist*, int);
long portRead(const ubx_port_t*, KDL::Wrench*, int);


// Templated makeSub and makePub functions
template<typename R, typename K>
ros::Subscriber makeSub(ros::NodeHandle *nh,
                        const struct ubxros_conn *uc,
                        const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<R>(
        uc->topic,
        uc->queue_size,
        [p_sub](const boost::shared_ptr<R const>& msg) {
            K v;
            convert(*msg, v);
            portWrite(p_sub, &v, 1);
        });
}

template<typename R, typename K>
std::function<void ()> makePub(ros::NodeHandle *nh,
                               const struct ubxros_conn *uc,
                               const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<R>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               R msg;
               K v;

               int len = portRead(p_pub, &v, 1);

               convert(v, msg);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing data on topic %s", uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}
