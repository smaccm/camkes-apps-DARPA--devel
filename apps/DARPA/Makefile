#
# Copyright 2014, NICTA
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(NICTA_BSD)
#

TARGETS := $(notdir ${SOURCE_DIR}).cdl
ADL := DARPA.camkes

client_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/client/include/*.h))
client_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/client/src/*.c))
    
canbus_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/canbus/include/*.h))
canbus_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/canbus/src/*.c))
    
can_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/can/include/*.h))
can_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/can/src/*.c))

uart_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/uart/include/*.h))
uart_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/uart/src/*.c))
    
spi_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi/include/*.h))
spi_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi/src/*.c))
    
clk_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/clk/include/*.h))
clk_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/clk/src/*.c))
    
gpio_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gpio/include/*.h))
gpio_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gpio/src/*.c))
    
timer_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/timer/include/*.h))
timer_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/timer/src/*.c))

gyro_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gyro/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
gyro_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gyro/src/*.c))

#mmc_HFILES := \
#    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/mmc/include/*.h))
#mmc_CFILES := \
#    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/mmc/src/*.c))

uart_LIBS += platsupport
gpio_LIBS += platsupport
spi_LIBS += platsupport
clk_LIBS += platsupport
timer_LIBS += platsupport

include ${PWD}/tools/camkes/camkes.mk
