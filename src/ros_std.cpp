#include "ros_std.hpp"

// define overloaded accessor functions portRead/portWrite/configGet
// (but don't define the basic accessor which are already defined in
// core ubx.
gen_class_accessors(float, float, float);
gen_class_accessors(double, double, double);
gen_class_accessors(int32, int32_t, int32_t);
gen_class_accessors(int64, int64_t, int64_t);
gen_class_accessors(uint32, uint32_t, uint32_t);
gen_class_accessors(uint64, uint64_t, uint64_t);
