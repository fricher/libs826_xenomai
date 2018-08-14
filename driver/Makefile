KDIR = /lib/modules/$(shell uname -r)/build

all: modules

obj-m := s826_xeno.o

EXTRA_CFLAGS += -Wno-unused -DS826_DRIVER_BUILD

modules:
	$(MAKE) -Wall -C $(KDIR) M=$(PWD) $@

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) $@

install: modules_install unload
	modprobe s826_xeno

load:
	modprobe s826_xeno

unload:
	modprobe -r -q s826_xeno


root_check:
	@id -u | grep "^0" || { \
		echo "Try again as root or use 'sudo make install'"; false; \
	}

modules_install: root_check
	$(MAKE) -C $(KDIR) M=$(PWD) $@
	depmod $(shell uname -r)

uninstall: root_check
	-modprobe -q -r s826_xeno
	rm -vf /lib/modules/$(shell uname -r)/extra/s826_xeno.ko
	depmod $(shell uname -r)
