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
    { .name="topic", .doc="ROS topic to publish to", .type_name = "char" },
    { .name="queue_size", .doc="outgoing message queue size", .type_name = "unsigned int" },
    { .name="latch", .doc="save message and send to future subscribers", .type_name = "int" },
    { 0 },
};

/* declaration port block ports */
ubx_port_t ubxros_ports[] = {
    { .name="pub", .doc="data to publish to topic", .in_type_name="int32_t" },
    { 0 },
};

/* instance state */
struct ubxros_info
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
int ubxros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ubxros_info *inf;

    inf = new ubxros_info();

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

/* start */
int ubxros_start(ubx_block_t *b)
{
    const uint32_t *queue_size;
    const int *latch;
    long len;

    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

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

    inf->pub = inf->nh->advertise<std_msgs::Int32>(inf->topic, inf->queue_size, inf->latch);

    /* OK */
    return 0;
}

/* stop */
void ubxros_stop(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;
    ubx_debug(b, "shutting down pub topic %s", inf->topic);
    inf->pub.shutdown();
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
template <typename T, long(*rd)(const ubx_port_t*,int32_t*)>
void ubxros_step(ubx_block_t *b)
{
    long len;
    T msg;

    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    // process callbacks
    ros::spinOnce();

    // publish data
    len = rd(inf->p_pub, &msg.data);

    if(len > 0) {
        inf->pub.publish(msg);
    } else if (len == 0) {
        ubx_debug(b, "no new data");
    } else {
        ubx_err(b, "failed to read toros port");
    }
}

// template void ubxros_step<std_msgs::Int32, ;


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
    .step = ubxros_step<std_msgs::Int32, &read_int32>,
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
