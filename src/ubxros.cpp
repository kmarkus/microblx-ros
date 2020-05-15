
#include <ubx/ubx.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

#include "types/ubxros_conn.h"
#include "types/ubxros_conn.h.hexarr"

/* types */
#include "ros_kdl.hpp"
#include "ros_std.hpp"

/* block meta information */
static char ubxros_meta[] =
    " { doc='A bridge block to connect to ROS', realtime=false }";


#define CONNECTIONS	"connections"
/* declaration of block configuration */
static ubx_config_t ros_config[] = {
    { .name=CONNECTIONS, .doc="microblx-ROS connections", .type_name = "struct ubxros_conn" },
    { 0 },
};

/* types defined by ptrig block */
ubx_type_t ubxros_types[] = {
	def_struct_type(struct ubxros_conn, &ubxros_conn_h),
	{ 0 },
};

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

ros::Publisher makeInt32Pub(ros::NodeHandle *nh,
                            const struct ubxros_conn *uc,
                            const ubx_port_t *p_pub)
{
    return nh->advertise<std_msgs::Int32>(uc->topic, uc->queue_size, uc->latch);
}


/* this is used to declare the type that ubxros can handle
 * for now it's just a static compile time list. Could be extended to
 * with dynamic loadable plugins..
 */
struct ubxros_handler {
    char *ros_type;
    char *ubx_type;

    ros::Subscriber (*subfact)(ros::NodeHandle *nh,
                               const struct ubxros_conn *uc,
                               const ubx_port_t *p_sub);

    ros::Publisher (*pubfact)(ros::NodeHandle *nh,
                              const struct ubxros_conn *uc,
                              const ubx_port_t *p_pub);
};

struct ubxros_handler handlers [] =
{
    {
        .ros_type = "std_msgs/Int32", .ubx_type = "int32_t",
        .subfact = makeInt32Sub, .pubfact = makeInt32Pub
    },
};



/* block state */
struct ubxros_info
{
    ros::NodeHandle *nh;

    struct ubxros_conn *conns; /* ROS connections */
};

static int ros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ubxros_info *inf;

    inf = new ubxros_info();

    if (inf == NULL) {
        ubx_err(b, "ubxros: failed to alloc memory");
        ret = EOUTOFMEM;
        goto out;
    }

    b->private_data = inf;

    len = cfg_getptr_ubxros_conns(CONNECTIONS, &inf->conns);
    assert(len>0);

    if (len == 0) {
        ubx_warn(b, "config connections unset");
        goto out_free;
    }

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
static int ros_start(ubx_block_t *b)
{
    long len;

    struct ros_info *inf = (struct ros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        return -1;
    }

    /* create publishers and subscribers

    /* OK */
    return 0;
}

/* stop */
static void ros_stop(ubx_block_t *b)
{
    struct ros_info *inf = (struct ros_info*) b->private_data;
    // ubx_debug(b, "shutting down %s", inf->topic);
    // inf->pub.shutdown();
    // inf->sub.shutdown();
}

/* cleanup */
static void ros_cleanup(ubx_block_t *b)
{
    struct ros_info *inf = (struct ros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    delete(inf);

    ros::shutdown();

    return;
}

/* step */
static void ros_step(ubx_block_t *b)
{
    long len;
    T msg;

    struct ros_info *inf = (struct ros_info*) b->private_data;

    // process callbacks
    ros::spinOnce();

    // publish data
    len = portRead(inf->p_pub, &msg.data);

    if(len > 0) {
        inf->pub.publish(msg);
    } else if (len == 0) {
        ubx_debug(b, "no new data");
    } else {
        ubx_err(b, "failed to read toros port");
    }
}

/* put everything together */
ubx_block_t ubxros_block =
{
    .name = "ubxros",
    .meta_data = ubxros_meta,
    .type = BLOCK_TYPE_COMPUTATION,
    .configs = ros_config,
    .init = ros_init,
    .start = ros_start,
    .stop = ros_stop,
    .cleanup = ros_cleanup,
    .step = ros_step,
};


int ubxros_mod_init(ubx_node_info_t* ni)
{
    for (tptr = ubxros_types; tptr->name != NULL; tptr++) {
        ret = ubx_type_register(ni, tptr);
        if (ret != 0) {
            ubx_log(UBX_LOGLEVEL_ERR, ni,
                    __func__,
                    "failed to register type %s", tptr->name);
            goto out;
        }
    }
    ret = ubx_block_register(ni, ubxros_block);
    if (ret != 0) {
        ubx_log(UBX_LOGLEVEL_ERR, ni, __func__,
                "failed to register %s block", ubxros_block->name);
    }

out:
    return ret;
}

void ubxros_mod_cleanup(ubx_node_info_t *ni)
{
    ubx_type_t *tptr;

    for (tptr = ptrig_types; tptr->name != NULL; tptr++)
        ubx_type_unregister(ni, tptr->name);

    ubx_block_unregister(ni, ubxros_block->name);
}

UBX_MODULE_INIT(ubxros_mod_init)
UBX_MODULE_CLEANUP(ubxros_mod_cleanup)
UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)
