##############################################################################
# Copyright (C) 2010 Schweitzer Engineering Laboratories, Inc.
#
# Makefile for the selserial kernel module 
##############################################################################

# Backwards compatibility for older Kbuild releases that don't look for a 
# Kbuild file
ifneq ($(KERNELRELEASE),)
include Kbuild
else

# KERNELDIR must be automatically detected or set by the user in order to 
# get the correct headers
ifndef KERNELDIR
ifndef KVERS
   KERNELDIR := /lib/modules/$(shell uname -r)/build
else
   KERNELDIR := /usr/src/linux-headers-$(KVERS)
endif
endif
ifneq ($(shell if test -d $(KERNELDIR); then echo yes; fi),yes)
   $(error Error: Unable to find the sources at $(KERNELDIR) \
                  Set KERNELDIR=<directory> and run Make again)
endif

KCONFIG_OPTS :=
KCONFIG_OPTS += CONFIG_SEL=m
KCONFIG_OPTS += CONFIG_SEL_IRIG=m
KCONFIG_OPTS += CONFIG_SEL_SELSERIAL=m
KCONFIG_OPTS += CONFIG_SEL_SELSERIAL_STALE_DATA_TIMER=n

LOCAL_CFLAGS := -I$(CURDIR)/include
LOCAL_CFLAGS += -DCONFIG_SEL
LOCAL_CFLAGS += -DCONFIG_SEL_IRIG
LOCAL_CFLAGS += -DCONFIG_SEL_SELSERIAL
KCONFIG_OPTS += EXTRA_CFLAGS="$(LOCAL_CFLAGS)"

modules::
	 $(MAKE) -C $(KERNELDIR) M=$(CURDIR) $(KCONFIG_OPTS) modules

modules_install:: modules
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) INSTALL_MOD_PATH=$(MODPREFIX) $(KCONFIG_OPTS) modules_install

clean::
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) $(KCONFIG_OPTS) clean

endif # ifneq ($(KERNELRELEASE),)
