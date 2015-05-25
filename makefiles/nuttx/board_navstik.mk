#
# Board-specific definitions for the NAVSTIK NXT
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = NAVSTIK

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
