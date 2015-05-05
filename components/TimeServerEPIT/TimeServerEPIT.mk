CURRENT_DIR := $(dir $(abspath $(lastword ${MAKEFILE_LIST})))

TimeServerEPIT_CFILES := $(wildcard ${CURRENT_DIR}/src/*.c)

TimeServerEPIT_LIBS := platsupport

