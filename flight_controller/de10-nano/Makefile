# Main flight controller:
MAIN?=Flight_controller_v2

# Test code:
# MAIN?=sensors_tests/motor_test
# MAIN?=sensors_tests/analog_test
# MAIN?=FC_fixed-point-tester

# Other functionalities:
# MAIN?=sensors_tests/esc_calibration
# MAIN?=simulation/simulation

SERIAL?=/dev/ttyUSB0

# Do not modify any further
I2C?=basic_lib/i2c_master
GPS?=basic_lib/gps
LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(GPS).c $(MAIN).c -o de10-nano.elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(I2C).c $(GPS).c $(MAIN).c -o ~/t-crest/patmos/tmp/de10-nano.elf -lm
download:
	patserdow -v $(SERIAL) de10-nano.elf

