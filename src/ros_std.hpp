#include <ubx/ubx.h>
#include "types/ubxros_conn.h"

#include <ros/ros.h>
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

ros::Subscriber makeFloat32Sub(ros::NodeHandle *nh,
                               const struct ubxros_conn *uc,
                               const ubx_port_t *p_sub);

std::function<void()> makeFloat32Pub(ros::NodeHandle *nh,
                                     const struct ubxros_conn *uc,
                                     const ubx_port_t *p_pub);

ros::Subscriber makeFloat64Sub(ros::NodeHandle *nh,
                               const struct ubxros_conn *uc,
                               const ubx_port_t *p_sub);

std::function<void()> makeFloat64Pub(ros::NodeHandle *nh,
                                     const struct ubxros_conn *uc,
                                     const ubx_port_t *p_pub);

ros::Subscriber makeInt32Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub);

std::function<void()> makeInt32Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub);

ros::Subscriber makeInt64Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub);

std::function<void()> makeInt64Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub);


ros::Subscriber makeUInt32Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub);

std::function<void()> makeUInt32Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub);


ros::Subscriber makeUInt64Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub);

std::function<void()> makeUInt64Pub(ros::NodeHandle *nh,
                                   const struct ubxros_conn *uc,
                                   const ubx_port_t *p_pub);
