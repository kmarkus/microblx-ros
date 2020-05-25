#define UBX_DEBUG

#include "ros_std.hpp"

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

gen_class_accessors(float, float, float);
gen_class_accessors(double, double, double);
gen_class_accessors(int32, int32_t, int32_t);
gen_class_accessors(int64, int64_t, int64_t);
gen_class_accessors(uint32, uint32_t, uint32_t);
gen_class_accessors(uint64, uint64_t, uint64_t);

//
// Concrete type handers
//

// Float32
ros::Subscriber makeFloat32Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::Float32>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::Float32ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %f", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeFloat32Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::Float32>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::Float32 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %f on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}

// Float64
ros::Subscriber makeFloat64Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::Float64>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::Float64ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %f", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeFloat64Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::Float64>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::Float64 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %f on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}



// Int32
ros::Subscriber makeInt32Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::Int32>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::Int32ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %i", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeInt32Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::Int32>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::Int32 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %i on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}

// Int64
ros::Subscriber makeInt64Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::Int64>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::Int64ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %li", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeInt64Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::Int64>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::Int64 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %li on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}

// UInt32
ros::Subscriber makeUInt32Sub(ros::NodeHandle *nh,
                              const struct ubxros_conn *uc,
                              const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::UInt32>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::UInt32ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %i", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeUInt32Pub(ros::NodeHandle *nh,
                                    const struct ubxros_conn *uc,
                                    const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::UInt32>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::UInt32 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %i on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}

// UInt64
ros::Subscriber makeUInt64Sub(ros::NodeHandle *nh,
                              const struct ubxros_conn *uc,
                              const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<std_msgs::UInt64>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::UInt64ConstPtr& msg) {
            ubx_debug(p_sub->block, "received %li", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

std::function<void()> makeUInt64Pub(ros::NodeHandle *nh,
                                    const struct ubxros_conn *uc,
                                    const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<std_msgs::UInt64>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               std_msgs::UInt64 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing %li on topic %s", msg.data, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}
