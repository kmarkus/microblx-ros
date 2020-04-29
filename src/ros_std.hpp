#include <ubx/ubx.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

long portRead(const ubx_port_t *p, char *x, const int len);
long portWrite(const ubx_port_t *p, const char *x, const int len);

long portRead(const ubx_port_t *p, float *x, const int len);
long portWrite(const ubx_port_t *p, const float *x, const int len);

long portRead(const ubx_port_t *p, double *x, const int len);
long portWrite(const ubx_port_t *p, const double *x, const int len);

long portRead(const ubx_port_t *p, uint32_t *x, const int len);
long portRead(const ubx_port_t *p, uint64_t *x, const int len);
long portRead(const ubx_port_t *p, int32_t *x, const int len);
long portRead(const ubx_port_t *p, int64_t *x, const int len);

long portWrite(const ubx_port_t *p, const int32_t *x, const int len);
long portWrite(const ubx_port_t *p, const int64_t *x, const int len);
long portWrite(const ubx_port_t *p, const uint32_t *x, const int len);
long portWrite(const ubx_port_t *p, const uint64_t *x, const int len);
