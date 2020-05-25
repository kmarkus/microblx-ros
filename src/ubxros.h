#define UBX_DEBUG

#include <ubx/ubx.h>
#include <ros/ros.h>

#include "types/ubxros_conn.h"
#include "types/ubxros_conn.h.hexarr"

#define DOCSTR_MAXLEN	128

// this is used to declare the type that ubxros can handle for now
// it's just a static compile time list. Could be extended to with
// dynamic loadable plugins.
struct ubxros_handler {
    const char *ros_type;
    const char *ubx_type;

    // a factory for creating a Subscription to the topic described by
    // uc. Will setup a callback for publishing received values on
    // port p_sub
    ros::Subscriber (*subfact)(ros::NodeHandle *nh,
                               const struct ubxros_conn *uc,
                               const ubx_port_t *p_sub);


    // a factory for creating publisher functions. A publisher
    // function is a thunk that will read from the given port and
    // publish it onto the given topic.
    std::function<void()> (*pubfact)(ros::NodeHandle *nh,
                                     const struct ubxros_conn *uc,
                                     const ubx_port_t *p_pub);

};

// state of an active ubxros connection
struct conn_state {
    struct ubxros_conn *conn;
    ros::Subscriber sub;
    std::function<void ()> pub;
};

// Block state
struct ubxros_info
{
    ros::NodeHandle *nh;
    // ptr to configs
    const struct ubxros_conn *conn_cfg;
    long conn_len;

    struct conn_state *conn_state;
};


//
// Block definitions
//

// block meta information
char ubxros_meta[] =
    "{ doc='A generic mult-type bridge block to connect to ROS', realtime=false }";


// block configs
const char* CONNECTIONS = "connections";

ubx_config_t ubxros_config[] = {
    { .name=CONNECTIONS, .doc="microblx-ROS connections", .type_name = "struct ubxros_conn" },
    { 0 },
};

// ubxros types and accessors
ubx_type_t ubxros_types[] = {
    def_struct_type(struct ubxros_conn, &ubxros_conn_h),
    { 0 },
};

def_cfg_getptr_fun(cfg_getptr_ubxros_conn, struct ubxros_conn);
