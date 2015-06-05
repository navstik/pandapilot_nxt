#
# Board-specific startup code for the NAVSTIK NXT
#

SRCS		 = navstik_can.c \
		   navstik_init.c \
		   navstik_pwm_servo.c \
		   navstik_spi.c \
		   navstik_usb.c \
		   navstik_led.c

MAXOPTIMIZATION	 = -Os
