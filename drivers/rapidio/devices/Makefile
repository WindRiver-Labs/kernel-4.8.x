#
# Makefile for RapidIO devices
#

obj-$(CONFIG_RAPIDIO_TSI721)	+= tsi721_mport.o
tsi721_mport-y			:= tsi721.o
tsi721_mport-$(CONFIG_RAPIDIO_DMA_ENGINE) += tsi721_dma.o
obj-$(CONFIG_RAPIDIO_MPORT_CDEV) += rio_mport_cdev.o
obj-$(CONFIG_TI_KEYSTONE_RAPIDIO) += keystone_rio_mport.o
keystone_rio_mport-y            := keystone_rio.o keystone_rio_serdes.o \
				keystone_rio_mp.o
keystone_rio_mport-$(CONFIG_RAPIDIO_DMA_ENGINE) += keystone_rio_dma.o
