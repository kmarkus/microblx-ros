#undef UBX_DEBUG

#include "ubxros.h"

#include "ros_std.hpp"
#include "ros_kdl.hpp"


struct ubxros_handler handlers [] = {
    {
        .ros_type = "std_msgs/Float32",
        .ubx_type = "float",
        .subfact = makeSub<std_msgs::Float32>,
        .pubfact = makePub<std_msgs::Float32>,
    }, {
        .ros_type = "std_msgs/Float64",
        .ubx_type = "double",
        .subfact = makeSub<std_msgs::Float64>,
        .pubfact = makePub<std_msgs::Float64>,
    }, {
        .ros_type = "std_msgs/Int32",
        .ubx_type = "int32_t",
        .subfact = makeSub<std_msgs::Int32>,
        .pubfact = makePub<std_msgs::Int32>,
    }, {
        .ros_type = "std_msgs/Int64",
        .ubx_type = "int64_t",
        .subfact = makeSub<std_msgs::Int64>,
        .pubfact = makePub<std_msgs::Int64>,
    }, {
        .ros_type = "std_msgs/UInt32",
        .ubx_type = "uint32_t",
        .subfact = makeSub<std_msgs::UInt32>,
        .pubfact = makePub<std_msgs::UInt32>,
    }, {
        .ros_type = "std_msgs/UInt64",
        .ubx_type = "uint64_t",
        .subfact = makeSub<std_msgs::UInt64>,
        .pubfact = makePub<std_msgs::UInt64>,
#ifdef CONFIG_GEOMETRY_MSGS
    }, {
        .ros_type = "geometry_msgs/Vector3",
        .ubx_type = "struct kdl_vector",
        .subfact = makeSub<geometry_msgs::Vector3, KDL::Vector>,
        .pubfact = makePub<geometry_msgs::Vector3, KDL::Vector>,
    }, {
        .ros_type = "geometry_msgs/Quaternion",
        .ubx_type = "struct kdl_rotation",
        .subfact = makeSub<geometry_msgs::Quaternion, KDL::Rotation>,
        .pubfact = makePub<geometry_msgs::Quaternion, KDL::Rotation>,
    }, {
        .ros_type = "geometry_msgs/Transform",
        .ubx_type = "struct kdl_frame",
        .subfact = makeSub<geometry_msgs::Transform, KDL::Frame>,
        .pubfact = makePub<geometry_msgs::Transform, KDL::Frame>,
    }, {
        .ros_type = "geometry_msgs/Pose",
        .ubx_type = "struct kdl_frame",
        .subfact = makeSub<geometry_msgs::Pose, KDL::Frame>,
        .pubfact = makePub<geometry_msgs::Pose, KDL::Frame>,
    }, {
        .ros_type = "geometry_msgs/Twist",
        .ubx_type = "struct kdl_twist",
        .subfact = makeSub<geometry_msgs::Twist, KDL::Twist>,
        .pubfact = makePub<geometry_msgs::Twist, KDL::Twist>,
    }, {
        .ros_type = "geometry_msgs/Wrench",
        .ubx_type = "struct kdl_wrench",
        .subfact = makeSub<geometry_msgs::Wrench, KDL::Wrench>,
        .pubfact = makePub<geometry_msgs::Wrench, KDL::Wrench>,
#endif
    }
};

const struct ubxros_handler* get_handler(const struct ubxros_conn* conn)
{
    for (unsigned long i=0; i<ARRAY_SIZE(handlers); i++) {

        if (strnlen(conn->ubx_type, TYPE_NAME_MAXLEN) > 0) {
            if (strncmp(handlers[i].ubx_type, conn->ubx_type, TYPE_NAME_MAXLEN) != 0)
                continue;
        }

        if (strnlen(conn->ros_type, TYPE_NAME_MAXLEN) > 0) {
            if (strncmp(handlers[i].ros_type, conn->ros_type, TYPE_NAME_MAXLEN) != 0)
                continue;
        }
        return &handlers[i];
    }
    return NULL;
}

int check_connections(ubx_block_t *b)
{
    int ret = 0;

    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    if (inf->conn_len == 0)
        ubx_warn(b, "no connections configured");

    for (long i=0; i<inf->conn_len; i++) {
        const char dir = inf->conn_cfg[i].dir[0];
        const char *ubx_type = inf->conn_cfg[i].ubx_type;
        const char *ros_type = inf->conn_cfg[i].ros_type;
        const char *topic = inf->conn_cfg[i].topic;
        const struct ubxros_handler* handler;

        handler = get_handler(&inf->conn_cfg[i]);

        if (!handler) {
            ubx_err(b, "EINVALID_TYPE: no handler for types %s/%s (topic %s)",
                    (ubx_type != NULL) ? ubx_type : "-",
                    (ros_type != NULL) ? ros_type : "-", topic);
            ret = EINVALID_TYPE;
        }

        if (strnlen(topic, ROS_TOPIC_MAXLEN) == 0) {
            ubx_err(b, "EINVALID_CONFIG: missing topic name for connection #%li", i);
            ret = EINVALID_CONFIG;
        }

        if (dir != 'P' && dir != 'S') {
            ubx_err(b, "EINVALID_CONFIG: invalid dir %c of connection %li", dir, i);
            ret = EINVALID_CONFIG;
        }

        if (dir == 'S' && inf->conn_cfg[i].latch != 0) {
            ubx_warn(b, "EINVALID_CONFIG: setting latch in SUB connection %li has no effect", i);
        }
    }

    return ret;
}


// init
int ubxros_init(ubx_block_t *b)
{
    int ret = -1;
    struct ubxros_info *inf;
    char docstr[DOCSTR_MAXLEN];


    for (unsigned long i=0; i<ARRAY_SIZE(handlers); i++)
        ubx_debug(b, "handler %li: %s <-> %s",
                  i, handlers[i].ros_type, handlers[i].ubx_type);

    try {
        inf = new ubxros_info();
    }
    catch (const std::bad_alloc& e) {
        ubx_err(b, "ubxros: failed to alloc block state");
        return EOUTOFMEM;
    }

    b->private_data = inf;

    inf->conn_len = cfg_getptr_ubxros_conn(b, CONNECTIONS, &inf->conn_cfg);
    assert(inf->conn_len > 0);

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

    try {
        inf->nh = new ros::NodeHandle();
    } catch (const std::bad_alloc& e) {
        ubx_err(b, "ubxros: failed to alloc NodeHandle");
        ret = EOUTOFMEM;
        goto out_free;
    }

    // ensure a ROS master is running
    if (!ros::master::check()) {
        ubx_err(b, "no ROS master found");
        goto out_free;
    }

    // create ports
    for (int i=0; i<inf->conn_len; i++) {
        const char dir = inf->conn_cfg[i].dir[0];
        const char *topic = inf->conn_cfg[i].topic;
        const ubxros_handler *handler = get_handler(&inf->conn_cfg[i]);

        if (dir == 'P') {
            snprintf(docstr, DOCSTR_MAXLEN, "in-port to publish on topic %s", topic);

            ret = ubx_inport_add(b, topic, docstr, handler->ubx_type, 1);

            if (ret)
                goto out_rmports;

        } else if (dir == 'S') {
            snprintf(docstr, DOCSTR_MAXLEN, "out-port with data from topic %s", topic);

            ret = ubx_outport_add(b, topic, docstr, handler->ubx_type, 1);

            if (ret)
                goto out_rmports;
        }

        ubx_debug(b, "added %s-port %s [%s]", (dir=='P')?"out":"in", topic, handler->ubx_type);
    }

    /* all good */
    ret=0;
    goto out;

out_rmports:
    for (int i=0; i<inf->conn_len; i++) {
        const char *topic = inf->conn_cfg[i].topic;
        if(ubx_port_get(b, topic) != 0)
            ubx_port_rm(b, topic);
    }

out_free:
    delete(inf);
out:
    return ret;
}

// start
int ubxros_start(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    if (!ros::ok()) {
        ubx_err(b, "ROS node not initialized or shutting down");
        return -1;
    }

    try {
        inf->conn_state = new struct conn_state[inf->conn_len];
    }
    catch (const std::bad_alloc& e) {
        ubx_err(b, "EOUTOFMEM: failed to alloc conn_state");
        return EOUTOFMEM;
    }

    // create publishers and subscribers
    for (int i=0; i<inf->conn_len; i++) {
        const char dir = inf->conn_cfg[i].dir[0];
        const char *topic = inf->conn_cfg[i].topic;
        const ubxros_handler *handler = get_handler(&inf->conn_cfg[i]);
        const ubx_port_t *p = ubx_port_get(b, topic);

        ubx_info(b, "%s: topic %s [%s] %s ubx %s [%s]",
                 (dir == 'P' ? "pub" : "sub"),
                 topic, handler->ros_type,
                 (dir == 'P' ? "<-" : "->"),
                 topic, handler->ubx_type);

        if (dir == 'P') {
            inf->conn_state[i].pub =
                handler->pubfact(inf->nh, &inf->conn_cfg[i], p);
        } else if (dir == 'S') {
            inf->conn_state[i].sub =
                handler->subfact(inf->nh, &inf->conn_cfg[i], p);
        }
    }

    // OK
    return 0;
}

// stop
void ubxros_stop(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    for (int i=0; i<inf->conn_len; i++) {

        const char dir = inf->conn_cfg[i].dir[0];

        if (dir == 'S')
            inf->conn_state[i].sub.shutdown();

        // the captured pub will be deleted when the capturing
        // function is deleted below.
    }

    delete[] inf->conn_state;

    // remove all added ports
    for (int i=0; i<inf->conn_len; i++)
        ubx_port_rm(b, inf->conn_cfg[i].topic);

    inf->conn_state = NULL;
}

// cleanup
void ubxros_cleanup(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    ubx_info(b, "shutting down ROS node %s", ros::this_node::getName().c_str());

    delete(inf->nh);
    delete(inf);

    ros::shutdown();

    return;
}

// step
void ubxros_step(ubx_block_t *b)
{
    struct ubxros_info *inf = (struct ubxros_info*) b->private_data;

    // process callbacks
    ros::spinOnce();

    // publish data
    for (int i=0; i<inf->conn_len; i++) {
        if (inf->conn_cfg[i].dir[0] == 'P')
            inf->conn_state[i].pub();
    }
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
