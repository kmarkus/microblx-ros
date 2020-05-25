#include <ubx/ubx.h>
#include <ros/ros.h>

#include "types/ubxros_conn.h"

ros::Subscriber makeKDLVectorSub(ros::NodeHandle *nh,
                                 const struct ubxros_conn *uc,
                                 const ubx_port_t *p_sub);

std::function<void ()> makeKDLVectorPub(ros::NodeHandle *nh,
                                        const struct ubxros_conn *uc,
                                        const ubx_port_t *p_pub);
