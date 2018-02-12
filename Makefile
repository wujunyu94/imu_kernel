obj-m := imu.o test.o
imu-objs :=IMU_SPI.o hrtimer.o

#obj-m := n_nvs.o w_nvs.o hrtimer.o spi_kernel.o
ifndef CROSS_COMPILE
KDIR := /lib/modules/$(shell uname -r)/build
else
KDIR := ~/am335x_kernel/
endif


PWD := $(shell pwd)
all:
	make -C $(KDIR) SUBDIRS=$(PWD)  modules

clean:
	rm *.o *.ko *.symvers *.order *.mod.c 
