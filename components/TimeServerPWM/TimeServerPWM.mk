CURRENT_DIR := $(dir $(abspath $(lastword ${MAKEFILE_LIST})))

TimeServerPWM_CFILES := $(wildcard ${CURRENT_DIR}/src/*.c)

TimeServerPWM_LIBS := platsupport

