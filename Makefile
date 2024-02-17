#
# Out-of-tree Makefile for building driver.
#

obj-m += dma_manager.o
dma_manager-y := dma_manager_main.o zcdma.o

# Add the current directory to the include path
ccflags-y := -I$(src)

PWD ?= $(shell pwd)

ifdef KERNEL_SRC
    KERNEL_SRC_DIR := $(KERNEL_SRC)
else
    KERNEL_SRC_DIR ?= /lib/modules/$(shell uname -r)/build
endif

#
# For out of kernel tree rules
#
all:
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) clean
