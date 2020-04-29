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
    { .name="topic", .doc="ROS topic to publish to", .type_name = "char" },
    { .name="queue_size", .doc="outgoing message queue size", .type_name = "unsigned int" },
    { .name="latch", .doc="save message and send to future subscribers", .type_name = "int" },
    { 0 },
};

/* declaration port block ports */
ubx_port_t ros_ports[] = {
    { .name="pub", .doc="data to publish to topic", .in_type_name="int32_t" },
    { 0 },
};

/* instance state */
struct ros_info
{
    ros::NodeHandle *nh;
    ros::Publisher pub;

    int latch;
    uint32_t queue_size;

    const char *topic;
    ubx_port_t *p_pub;
};

/*
 * Generic functions
 */

/* init */
int ros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ros_info *inf;

    inf = new ros_info();

    if (inf == NULL) {
        ubx_err(b, "myblock: failed to alloc memory");
        ret = EOUTOFMEM;
        goto out;
    }

    b->private_data = inf;

    /* config topic_pub */
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
    inf->p_pub = ubx_port_get(b, "pub");
    assert(inf->p_pub);

    /* all good */
    ret=0;
    goto out;

out_free:
    delete(inf);
out:
    return ret;
}

/* stop */
void ros_stop(ubx_block_t *b)
{
    struct ros_info *inf = (struct ros_info*) b->private_data;
    ubx_debug(b, "shutting down pub topic %s", inf->topic);
    inf->pub.shutdown();
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

/* start */
template <typename T>
int ros_start(ubx_block_t *b)
{
    const uint32_t *queue_size;
    const int *latch;
    long len;

    struct ros_info *inf = (struct ros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        return -1;
    }

    len = cfg_getptr_uint32(b, "queue_size", &queue_size);
    assert(len>=0);
    inf->queue_size = (len == 0) ? 1 : *queue_size;

    len = cfg_getptr_int(b, "latch", &latch);
    assert(len>=0);
    inf->latch = (len == 0) ? 1 : *latch;

    ubx_debug(b, "pub topic: %s, queue_size: %ul, latch: %s",
              inf->topic, inf->queue_size, (inf->latch==0) ? "false":"true");

    inf->pub = inf->nh->advertise<T>(inf->topic, inf->queue_size, inf->latch);

    /* OK */
    return 0;
}

/* step */
template <typename T>
void ros_step(ubx_block_t *b)
{
    long len;
    T msg;

    struct ros_info *inf = (struct ros_info*) b->private_data;

    // process callbacks
    // TODO really need this for pub?
    ros::spinOnce();

    // publish data
    len = portRead(inf->p_pub, &msg.data, 1);

    if(len > 0) {
        inf->pub.publish(msg);
    } else if (len == 0) {
        ubx_debug(b, "no new data");
    } else {
        ubx_err(b, "failed to read toros port");
    }
}

/* put everything together */
__attribute__ ((visibility("default"))) ubx_block_t pub_blocks[] =
{
    {
        .name = "ros_int32_pub",
        .meta_data = ros_meta,
        .type = BLOCK_TYPE_COMPUTATION,
        .ports = ros_ports,
        .configs = ros_config,
        .init = ros_init,
        .start = ros_start<std_msgs::Int32>,
        .stop = ros_stop,
        .cleanup = ros_cleanup,
        .step = ros_step<std_msgs::Int32>,
    }, {
        .name = "ros_int64_pub",
        .meta_data = ros_meta,
        .type = BLOCK_TYPE_COMPUTATION,
        .ports = ros_ports,
        .configs = ros_config,
        .init = ros_init,
        .start = ros_start<std_msgs::Int64>,
        .stop = ros_stop,
        .cleanup = ros_cleanup,
        .step = ros_step<std_msgs::Int64>,
    }
};


int ros_mod_init(ubx_node_info_t* ni)
{
    int ret = 0;

    for(unsigned long i = 0; i < ARRAY_SIZE(pub_blocks); i++)
        ret |= ubx_block_register(ni, &pub_blocks[i]);

    return ret;
}

void ros_mod_cleanup(ubx_node_info_t *ni)
{

    for(unsigned long i = 0; i < ARRAY_SIZE(pub_blocks); i++)
        ubx_block_unregister(ni, pub_blocks[i].name);

}

UBX_MODULE_INIT(ros_mod_init)
UBX_MODULE_CLEANUP(ros_mod_cleanup)
UBX_MODULE_LICENSE_SPDX(MIT)
