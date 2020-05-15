#ifndef UBXROS_CONN_H
#define UBXROS_CONN_H

enum {
	ROS_TOPIC_MAXLEN = 256,
	ROS_MSG_ID_MAXLEN = 256,
};

/* description of one microblx to ROS connection */
struct ubxros_conn {
	char topic[ROS_TOPIC_MAXLEN];
	char dir[1];
	char ubx_type[TYPE_NAME_MAXLEN];

	int queue_size;
	int latch; /* pub only */

};

#endif /* UBXROS_CONN_H */
