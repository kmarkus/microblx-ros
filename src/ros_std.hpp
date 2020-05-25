#include <ubx/ubx.h>
#include "types/ubxros_conn.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

long portRead(const ubx_port_t *p, char *x, const int len);
long portRead(const ubx_port_t *p, float *x, const int len);
long portRead(const ubx_port_t *p, double *x, const int len);
long portRead(const ubx_port_t *p, int32_t *x, const int len);
long portRead(const ubx_port_t *p, int64_t *x, const int len);
long portRead(const ubx_port_t *p, uint32_t *x, const int len);
long portRead(const ubx_port_t *p, uint64_t *x, const int len);

long portWrite(const ubx_port_t *p, const char *x, const int len);
long portWrite(const ubx_port_t *p, const float *x, const int len);
long portWrite(const ubx_port_t *p, const double *x, const int len);
long portWrite(const ubx_port_t *p, const int32_t *x, const int len);
long portWrite(const ubx_port_t *p, const int64_t *x, const int len);
long portWrite(const ubx_port_t *p, const uint32_t *x, const int len);
long portWrite(const ubx_port_t *p, const uint64_t *x, const int len);

template <typename T>
ros::Subscriber makeSub(ros::NodeHandle *nh,
                        const struct ubxros_conn *uc,
                        const ubx_port_t *p_sub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_sub != NULL);

    return nh->subscribe<T>(
        uc->topic,
        uc->queue_size,
        [p_sub,uc](const boost::shared_ptr<T const> &msg) {
            ubx_debug(p_sub->block, "received msg on %s", uc->topic);
            portWrite(p_sub, &msg->data, 1);
        });
}

template <typename T>
std::function<void()> makePub(ros::NodeHandle *nh,
                              const struct ubxros_conn *uc,
                              const ubx_port_t *p_pub)
{
    assert(nh != NULL);
    assert(uc != NULL);
    assert(p_pub != NULL);

    ros::Publisher pub = nh->advertise<T>(uc->topic, uc->queue_size, uc->latch);

    return [p_pub,uc,pub]() {
               T msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0) {
                   ubx_debug(p_pub->block, "publishing msg on topic %s", uc->topic);
                   pub.publish(msg);
               } else if (len == 0) {
                   ubx_debug(p_pub->block, "no new data on topic %s", uc->topic);
               } else {
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
               }
           };
}
