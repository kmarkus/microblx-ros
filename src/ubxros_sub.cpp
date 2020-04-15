/*
 * ROS mixed port connector
 */

#define UBX_DEBUG

#include <ubx/ubx.h>
#include <ubxkdl.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <kdl_conversions/kdl_msg.h>

/* block meta information */
char ubxros_meta[] =
    " { doc='A mixed port block geometry_msg/Vector3' block,"
    "   realtime=false,"
    "}";

/* declaration of block configuration */
ubx_config_t ubxros_config[] = {
    { .name="topic", .doc="ROS topic too subcribe to", .type_name = "char" },
    { 0 },
};

/* declaration port block ports */
ubx_port_t ubxros_ports[] = {
    { .name="sub", .doc="data read from subscribed topic", .out_type_name="int32_t" },
    { 0 },
};

/* instance state */
struct ubxros_info
{
    ros::NodeHandle *nh;
    ros::Subscriber sub;

    const char *topic;

    ubx_port_t *p_sub;
};

/* init */
int ubxros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ubxros_info *inf;

    /* allocate memory for the block local state */
    inf = new ubxros_info();

    if (inf == NULL) {
        ubx_err(b, "myblock: failed to alloc memory");
        ret = EOUTOFMEM;
        goto out;
    }

    b->private_data = inf;

    /* config topic */
    len = cfg_getptr_char(b, "topic", &inf->topic);

    assert(len>0);

    if (len == 0) {
        ubx_err(b, "mandatory config 'topic' unset");
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

    /* cache ports */
    inf->p_sub = ubx_port_get(b, "sub");
    assert(inf->p_sub);

    /* all good */
    ret=0;
    goto out;

out_free:
    delete(inf);
out:
    return ret;
}

/* start */
int ubxros_start(ubx_block_t *b)
{
    int ret = -1;
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        goto out;
    }

    /* subscribe */
    inf->sub = inf->nh->subscribe<std_msgs::Int32>(
        inf->topic, 10,
        [b,inf](const std_msgs::Int32ConstPtr& msg) {
            ubx_debug(b, "received %i", msg->data);
            write_int32(inf->p_sub, &msg->data);
        });
    /* OK */
    ret = 0;
out:
    return ret;
}

/* stop */
void ubxros_stop(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;
    ubx_debug(b, "shutting down topic %s", inf->topic);
    inf->sub.shutdown();
}

/* cleanup */
void ubxros_cleanup(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    delete(inf);

    ros::shutdown();

    return;
}

/* step */
void ubxros_step(ubx_block_t *b)
{
    (void)b;

    ros::spinOnce();
}

/* put everything together */
ubx_block_t ubxros_block = {
    .name = "ubxros",
    .meta_data = ubxros_meta,
    .type = BLOCK_TYPE_COMPUTATION,

    .ports = ubxros_ports,
    .configs = ubxros_config,

    /* ops */
    .init = ubxros_init,
    .start = ubxros_start,
    .stop = ubxros_stop,
    .cleanup = ubxros_cleanup,
    .step = ubxros_step,
};

int ubxros_mod_init(ubx_node_info_t* ni)
{
    return ubx_block_register(ni, &ubxros_block);
}

void ubxros_mod_cleanup(ubx_node_info_t *ni)
{
    ubx_block_unregister(ni, "ubxros");
}

UBX_MODULE_INIT(ubxros_mod_init)
UBX_MODULE_CLEANUP(ubxros_mod_cleanup)
UBX_MODULE_LICENSE_SPDX(MIT)
