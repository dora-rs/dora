# Unbounded sequences of different types
bool[] bool_values
byte[] byte_values
char[] char_values
float32[] float32_values
float64[] float64_values
int8[] int8_values
uint8[] uint8_values
int16[] int16_values
uint16[] uint16_values
int32[] int32_values
uint32[] uint32_values
int64[] int64_values
uint64[] uint64_values
string[] string_values
BasicTypes[] basic_types_values
Constants[] constants_values
Defaults[] defaults_values
bool[] bool_values_default [false, true, false]
byte[] byte_values_default [0, 1, 255]
char[] char_values_default [0, 1, 127]
float32[] float32_values_default [1.125, 0.0, -1.125]
float64[] float64_values_default [3.1415, 0.0, -3.1415]
int8[] int8_values_default [0, 127, -128]
uint8[] uint8_values_default [0, 1, 255]
int16[] int16_values_default [0, 32767, -32768]
uint16[] uint16_values_default [0, 1, 65535]
int32[] int32_values_default [0, 2147483647, -2147483648]
uint32[] uint32_values_default [0, 1, 4294967295]
int64[] int64_values_default [0, 9223372036854775807, -9223372036854775808]
uint64[] uint64_values_default [0, 1, 18446744073709551615]
string[] string_values_default ["", "max value", "min value"]
# Regression test: check alignment of basic field after a sequence field is correct
int32 alignment_check
