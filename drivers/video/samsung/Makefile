#
# Makefile for the s3c framebuffer driver
#

ifeq ($(CONFIG_FB_S3C),y)
obj-y         += s3cfb.o
obj-$(CONFIG_ARCH_S5PV210)	+= s3cfb_fimd6x.o

obj-$(CONFIG_FB_S3C_LTE480WV)	+= s3cfb_lte480wv.o
obj-$(CONFIG_FB_S3C_TL2796)     += s3cfb_tl2796.o
obj-$(CONFIG_FB_S3C_INNOLUX430)   += s3cfb_innolux430.o
obj-$(CONFIG_FB_S3C_UT7GM)	  += s3cfb_ut7gm.o
obj-$(CONFIG_FB_S3C_UTAUO104)	  += s3cfb_auo104.o
obj-$(CONFIG_FB_S3C_UTLTD121)	  += s3cfb_ltd121.o
endif

ifeq ($(CONFIG_FB_S3C_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
