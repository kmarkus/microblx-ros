
#include <ubx/ubx.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

#include "types/ubxros_conn.h"
#include "types/ubxros_conn.h.hexarr"

// types
#include "ros_kdl.hpp"
#include "ros_std.hpp"

// ubxros handlers

typedef std::function<void ()> pubFunc;

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
    pubFunc (*pubfact)(ros::NodeHandle *nh,
                       const struct ubxros_conn *uc,
                       const ubx_port_t *p_pub);

};

//
// Block definitions
//

// block meta information
static char ubxros_meta[] =
    "{ doc='A generic mult-type bridge block to connect to ROS', realtime=false }";


const char* CONNECTIONS = "connections";

/* declaration of block configuration */
static ubx_config_t ubxros_config[] = {
    { .name=CONNECTIONS, .doc="microblx-ROS connections", .type_name = "struct ubxros_conn" },
    { 0 },
};

// ubxros types and accessors
ubx_type_t ubxros_types[] = {
    def_struct_type(struct ubxros_conn, &ubxros_conn_h),
    { 0 },
};

def_cfg_getptr_fun(cfg_getptr_ubxros_conn, struct ubxros_conn);


// Block state
struct ubxros_info
{
    ros::NodeHandle *nh;
    const struct ubxros_conn *conns; /* ROS connections */
    long conns_len;
};



// Concrete type handers
ros::Subscriber makeInt32Sub(ros::NodeHandle *nh,
                             const struct ubxros_conn *uc,
                             const ubx_port_t *p_sub)
{
    return nh->subscribe<std_msgs::Int32>(
        uc->topic,
        uc->queue_size,
        [p_sub](const std_msgs::Int32ConstPtr& msg) {
            ubx_debug(o_sub->block, "received %i", msg->data);
            portWrite(p_sub, &msg->data, 1);
        });
}

pubFunc makeInt32Pub(ros::NodeHandle *nh,
                     const struct ubxros_conn *uc,
                     const ubx_port_t *p_pub)
{
    ros::Publisher pub = nh->advertise<std_msgs::Int32>(uc->topic, uc->queue_size, uc->latch);

    return [&p_pub,pub]() {
               std_msgs::Int32 msg;
               int len = portRead(p_pub, &msg.data, 1);

               if(len > 0)
                   pub.publish(msg);
               else if (len == 0)
                   ubx_debug(p_pub->block, "no new data");
               else
                   ubx_err(p_pub->block, "failed to read port %s", p_pub->name);
           };
}


struct ubxros_handler handlers [] = {
    {
        .ros_type = "std_msgs/Int32",
        .ubx_type = "int32_t",
        .subfact = makeInt32Sub,
        .pubfact = makeInt32Pub
    },
};


int check_connections(ubx_block_t *b)
{
    int ret = 0;

    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    if (inf->conns_len == 0)
        ubx_warn(b, "no connections configured");

    for (long i=0; i<inf->conns_len; i++) {
        const char dir = toupper(inf->conns[i].dir);
        const char *ubx_type = inf->conns[i].ubx_type;

        if (dir != 'P' && dir != 'S') {
            ubx_err(b, "EINVALID_CONFIG: invalid dir %c of connection %li",
                    inf->conns[i].dir, i);
            ret = EINVALID_CONFIG;
        }

        if (ubx_type_get(b->ni, ubx_type) == NULL) {
            ubx_err(b, "EINVALID_TYPE: %s in connection %li", ubx_type, i);
            ret = EINVALID_TYPE;
        }

        if (dir == 'S' && inf->conns[i].latch != 0) {
            ubx_warn(b, "EINVALID_CONFIG: setting latch in SUB connection %li has no effect", i);
        }
    }

    return ret;
}


static int ubxros_init(ubx_block_t *b)
{
    int ret = -1;
    struct ubxros_info *inf;

    inf = new ubxros_info();

    if (inf == NULL) {
        ubx_err(b, "ubxros: failed to alloc memory");
        ret = EOUTOFMEM;
        goto out;
    }

    b->private_data = inf;

    inf->conns_len = cfg_getptr_ubxros_conn(b, CONNECTIONS, &inf->conns);
    assert(inf->conns_len > 0);

    ret = check_connections(b);

    if (ret != 0)
        goto out_free;

    /* need to create node? */
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "ubx",
                  ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);

        /* start the node */
        ros::start();
        ubx_info(b, "initialized ROS node %s", ros::this_node::getName().c_str());
    }

    inf->nh = new ros::NodeHandle();

    /* ensure a ROS master is running */
    if (!ros::master::check()) {
        ubx_err(b, "no ROS master found");
        goto out_free;
    }

    /* all good */
    ret=0;
    goto out;

out_free:
    delete(inf);
out:
    return ret;
}

/* start */
static int ubxros_start(ubx_block_t *b)
{
    // struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        return -1;
    }

    // create publishers and subscribers

    // OK
    return 0;
}

/* stop */
static void ubxros_stop(ubx_block_t *b)
{
    (void)b;

    //struct ubxros_info *inf = (struct ubxros_info*) b->private_data;
    ubx_debug(b, "shutting down");
    // ubx_debug(b, "shutting down %s", inf->topic);
    // inf->pub.shutdown();
    // inf->sub.shutdown();
}

/* cleanup */
static void ubxros_cleanup(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    delete(inf);

    ros::shutdown();

    return;
}

/* step */
static void ubxros_step(ubx_block_t *b)
{
    (void)b;

    // struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    // process callbacks
    ros::spinOnce();

    // publish data
    // TODO invoke all publishers
}

/* put everything together */
ubx_block_t ubxros_block =
{
    .name = "ubxros",
    .meta_data = ubxros_meta,
    .type = BLOCK_TYPE_COMPUTATION,
    .configs = ubxros_config,
    .init = ubxros_init,
    .start = ubxros_start,
    .stop = ubxros_stop,
    .cleanup = ubxros_cleanup,
    .step = ubxros_step,
};


int ubxros_mod_init(ubx_node_info_t* ni)
{
    int ret;
    ubx_type_t *tptr;

    for (tptr = ubxros_types; tptr->name != NULL; tptr++) {
        ret = ubx_type_register(ni, tptr);
        if (ret != 0) {
            ubx_log(UBX_LOGLEVEL_ERR, ni,
                    __func__,
                    "failed to register type %s", tptr->name);
            goto out;
        }
    }
    ret = ubx_block_register(ni, &ubxros_block);
    if (ret != 0) {
        ubx_log(UBX_LOGLEVEL_ERR, ni, __func__,
                "failed to register %s block", ubxros_block.name);
    }

out:
    return ret;
}

void ubxros_mod_cleanup(ubx_node_info_t *ni)
{
    ubx_type_t *tptr;

    for (tptr = ubxros_types; tptr->name != NULL; tptr++)
        ubx_type_unregister(ni, tptr->name);

    ubx_block_unregister(ni, ubxros_block.name);
}

UBX_MODULE_INIT(ubxros_mod_init)
UBX_MODULE_CLEANUP(ubxros_mod_cleanup)
UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)
