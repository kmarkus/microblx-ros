// common code for both pub and sub

#include <ubx/ubx.h>

gen_class_accessors(char, char, char);
gen_class_accessors(int8, int8_t, int8_t);
gen_class_accessors(int32, int32_t, int32_t);
gen_class_accessors(int64, int64_t, int64_t);
gen_class_accessors(uint8, uint8_t, uint8_t);
gen_class_accessors(uint32, uint32_t, uint32_t);
gen_class_accessors(uint64, uint64_t, uint64_t);
