#ifndef UBXROS_CONN_H
#define UBXROS_CONN_H

enum {
	ROS_TOPIC_MAXLEN = 256,
	ROS_MSG_ID_MAXLEN = 256,
};

/* description of one microblx to ROS connection */
struct ubxros_conn {
	char topic[ROS_TOPIC_MAXLEN];
	char dir[1+1];
	char ubx_type[UBX_TYPE_NAME_MAXLEN];
	char ros_type[UBX_TYPE_NAME_MAXLEN];

	uint32_t queue_size;
	int latch; /* pub only */

};

#endif /* UBXROS_CONN_H */
