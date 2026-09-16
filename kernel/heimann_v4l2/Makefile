obj-m += heimann_v4l2_drv.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all: module test_heimann_v4l2

module:
	make -C $(KDIR) M=$(PWD) ARCH=arm64 HOSTCC=gcc modules

test_heimann_v4l2: test_heimann_v4l2.c
	$(CC) -O2 -Wall -Wextra -o $@ $<

clean:
	make -C $(KDIR) M=$(PWD) ARCH=arm64 HOSTCC=gcc clean
	$(RM) test_heimann_v4l2
