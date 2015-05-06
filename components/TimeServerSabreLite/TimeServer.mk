CURRENT_DIR := $(dir $(abspath $(lastword ${MAKEFILE_LIST})))

include TimeServerEPIT/TimeServerEPIT.mk


