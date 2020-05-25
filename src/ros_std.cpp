#define UBX_DEBUG

#include "ros_std.hpp"

#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

gen_class_accessors(char, char, char);
gen_class_accessors(int8, int8_t, int8_t);
gen_class_accessors(int32, int32_t, int32_t);
gen_class_accessors(int64, int64_t, int64_t);
gen_class_accessors(uint8, uint8_t, uint8_t);
gen_class_accessors(uint32, uint32_t, uint32_t);
gen_class_accessors(uint64, uint64_t, uint64_t);

//
// Concrete type handers
//
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
    ros::Publisher pub = nh->advertise<std_msgs::Int32>(uc->topic, uc->queue_size, uc->latch);

    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

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
