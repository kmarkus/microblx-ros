#define UBX_DEBUG

#include <ubxkdl.hpp>
#include <geometry_msgs/Vector3.h>
#include <kdl_conversions/kdl_msg.h>

#include "ros_kdl.hpp"

//
// KDL types
//
ros::Subscriber makeKDLVectorSub(ros::NodeHandle *nh,
                                 const struct ubxros_conn *uc,
                                 const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<geometry_msgs::Vector3>(
        uc->topic,
        uc->queue_size,
        [p_sub](const geometry_msgs::Vector3ConstPtr& msg) {
            KDL::Vector v;
            ubx_debug(p_sub->block, "received Vector3 x=%f,y=%f,z=%f, ",
                      msg->x, msg->y, msg->z);
            tf::vectorMsgToKDL(*msg, v);
            portWrite(p_sub, &v, 1);
        });

}

std::function<void ()> makeKDLVectorPub(ros::NodeHandle *nh,
                                        const struct ubxros_conn *uc,
                                        const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<geometry_msgs::Vector3>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               geometry_msgs::Vector3 msg;
               KDL::Vector v;

               int len = portRead(p_pub, &v, 1);

               tf::vectorKDLToMsg(v, msg);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing Vector3 x=%f,y=%f,z=%f on topic %s",
                             msg.x, msg.y, msg.z, uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}
