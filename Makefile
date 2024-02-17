obj-m += dma_manager_main.o zcdma.o

PWD ?= $(shell pwd)
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
