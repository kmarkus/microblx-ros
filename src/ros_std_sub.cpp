/*
 * ROS mixed port connector
 */

#include "ros_std.hpp"

/* block meta information */
char ros_meta[] =
    " { doc='A mixed port block geometry_msg/Vector3' block,"
    "   realtime=false,"
    "}";

/* declaration of block configuration */
ubx_config_t ros_config[] = {
    { .name="topic", .doc="ROS topic too subcribe to", .type_name = "char" },
    { 0 },
};

/* declaration port block ports */
ubx_port_t ros_ports[] = {
    { .name="sub", .doc="data read from subscribed topic", .out_type_name="int32_t" },
    { 0 },
};

/* instance state */
struct ros_info
{
    ros::NodeHandle *nh;
    ros::Subscriber sub;

    const char *topic;

    ubx_port_t *p_sub;
};

/* init */
int ros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ros_info *inf;

    /* allocate memory for the block local state */
    inf = new ros_info();

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
int ros_start(ubx_block_t *b)
{
    int ret = -1;
    struct ros_info *inf = (struct ros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        goto out;
    }

    /* subscribe */
    inf->sub = inf->nh->subscribe<std_msgs::Int32>(
        inf->topic, 10,
        [b,inf](const std_msgs::Int32ConstPtr& msg) {
            ubx_debug(b, "received %i", msg->data);
            portWrite(inf->p_sub, &msg->data);
        });
    /* OK */
    ret = 0;
out:
    return ret;
}

/* stop */
void ros_stop(ubx_block_t *b)
{
    struct ros_info *inf = (struct ros_info*) b->private_data;
    ubx_debug(b, "shutting down topic %s", inf->topic);
    inf->sub.shutdown();
}

/* cleanup */
void ros_cleanup(ubx_block_t *b)
{
    struct ros_info *inf = (struct ros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    delete(inf);

    ros::shutdown();

    return;
}

/* step */
void ros_step(ubx_block_t *b)
{
    (void)b;

    ros::spinOnce();
}

/* put everything together */
ubx_block_t ros_block = {
    .name = "ubxros",
    .meta_data = ros_meta,
    .type = BLOCK_TYPE_COMPUTATION,

    .ports = ros_ports,
    .configs = ros_config,

    /* ops */
    .init = ros_init,
    .start = ros_start,
    .stop = ros_stop,
    .cleanup = ros_cleanup,
    .step = ros_step,
};

int ros_mod_init(ubx_node_info_t* ni)
{
    return ubx_block_register(ni, &ros_block);
}

void ros_mod_cleanup(ubx_node_info_t *ni)
{
    ubx_block_unregister(ni, "ubxros");
}

UBX_MODULE_INIT(ros_mod_init)
UBX_MODULE_CLEANUP(ros_mod_cleanup)
UBX_MODULE_LICENSE_SPDX(MIT)
