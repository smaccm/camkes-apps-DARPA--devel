CURRENT_DIR := $(dir $(abspath $(lastword ${MAKEFILE_LIST})))

include TimeServerPWM/TimeServerPWM.mk

