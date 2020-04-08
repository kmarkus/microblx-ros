/*
 * myblock microblx function block (autogenerated, don't edit)
 */

#define UBX_DEBUG

#include <ubx/ubx.h>
#include <kdlubx/kdl.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"

def_read_fun(read_int32, int32_t)
def_write_fun(write_int32, int32_t)

/* block meta information */
char ubxros_meta[] =
    " { doc='A mixed port block geometry_msg/Vector3' block,"
    "   realtime=false,"
    "}";

/* declaration of block configuration */
ubx_config_t ubxros_config[] = {
    { .name="topic_pub", .type_name = "char" },
    { .name="topic_sub", .type_name = "char" },
    { 0 },
};

/* declaration port block ports */
ubx_port_t ubxros_ports[] = {
    { .name="fromros", .doc="out port", .out_type_name="int32_t" },
    { .name="toros", .doc="in port", .in_type_name="int32_t" },
    { 0 },
};

/* instance state */
struct ubxros_info
{
    ros::NodeHandle *nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    const char *topic_pub;
    const char *topic_sub;

    /* cached ports */
    struct
    {
        ubx_port_t *toros;
        ubx_port_t *fromros;
    };
};

/* init */
int ubxros_init(ubx_block_t *b)
{
    int ret = -1;
    long len;

    struct ubxros_info *inf;

    /* allocate memory for the block local state */
    // inf = static_cast<ubxros_info*>(calloc(1, sizeof(struct
    // ubxros_info)));
    inf = new ubxros_info();

    if (inf == NULL) {
        ubx_err(b, "myblock: failed to alloc memory");
        ret = EOUTOFMEM;
        goto out;
    }

    b->private_data = inf;

    /* config topic_pub */
    len = cfg_getptr_char(b, "topic_pub", &inf->topic_pub);
    assert(len>0);

    if (len == 0) {
        ubx_err(b, "mandatory config 'topic_pub' unset");
        goto out_free;
    }

    /* config topic_sub */
    len = cfg_getptr_char(b, "topic_sub", &inf->topic_sub);
    assert(len>=0);

    if (len == 0) {
        ubx_err(b, "mandatory config 'topic_sub' unset");
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

    /* subscribe */

    /* cache ports */
    inf->toros = ubx_port_get(b, "toros");
    inf->fromros = ubx_port_get(b, "fromros");
    assert(inf->toros);
    assert(inf->fromros);

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
    ubx_debug(b, "advertisting topic %s", inf->topic_pub);

    inf->pub = inf->nh->advertise<std_msgs::Int32>(inf->topic_pub, 10);

    inf->sub = inf->nh->subscribe<std_msgs::Int32>(
        inf->topic_sub, 10,
        [&](const std_msgs::Int32ConstPtr& msg) {
            ubx_debug(b, "received something");
            int32_t data = msg->data;
            data = 99;
            write_int32(inf->fromros, &data);
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
    ubx_debug(b, "shutting down topic");
    inf->pub.shutdown();
}

/* cleanup */
void ubxros_cleanup(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    free(b->private_data);

    ros::shutdown();

    return;
}

/* step */
void ubxros_step(ubx_block_t *b)
{
    long len;
    std_msgs::Int32 msg;

    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    // process callbacks
    ros::spinOnce();

    // publish data
    len = read_int32(inf->toros, &msg.data);

    if(len > 0) {
        inf->pub.publish(msg);
    } else if (len == 0) {
        ubx_debug(b, "no new data");
    } else {
        ubx_err(b, "failed to read toros port");
    }
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
