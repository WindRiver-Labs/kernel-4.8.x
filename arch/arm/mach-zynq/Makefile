#
# Makefile for the linux kernel.
#

# Common support
obj-y				:= common.o slcr.o pm.o zynq_ocm.o
obj-$(CONFIG_SMP)		+= headsmp.o platsmp.o
obj-$(CONFIG_SUSPEND)		+= suspend.o
