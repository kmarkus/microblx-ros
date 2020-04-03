#include "ros_vector3.hpp"

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct myblock_info
{
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct myblock_port_cache ports;
};

/* init */
int myblock_init(ubx_block_t *b)
{
	int ret = -1;
	struct myblock_info *inf;

	/* allocate memory for the block local state */
	if ((inf = (myblock_info*) calloc(1, sizeof(struct myblock_info)))==NULL) {
		ubx_err(b, "myblock: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	update_port_cache(b, &inf->ports);
	ret=0;
out:
	return ret;
}

/* start */
int myblock_start(ubx_block_t *b)
{
	/* struct myblock_info *inf = (struct myblock_info*) b->private_data; */
        ubx_info(b, "%s", __FUNCTION__);
	int ret = 0;
	return ret;
}

/* stop */
void myblock_stop(ubx_block_t *b)
{
	/* struct myblock_info *inf = (struct myblock_info*) b->private_data; */
        ubx_info(b, "%s", __FUNCTION__);
}

/* cleanup */
void myblock_cleanup(ubx_block_t *b)
{
	/* struct myblock_info *inf = (struct myblock_info*) b->private_data; */
        ubx_info(b, "%s", __FUNCTION__);
	free(b->private_data);
}

/* step */
void myblock_step(ubx_block_t *b)
{
	/* struct myblock_info *inf = (struct myblock_info*) b->private_data; */
        ubx_info(b, "%s", __FUNCTION__);
}

