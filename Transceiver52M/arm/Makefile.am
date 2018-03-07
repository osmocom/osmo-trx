if ARCH_ARM_A15
ARCH_FLAGS = -mfpu=neon-vfpv4
else
ARCH_FLAGS = -mfpu=neon
endif

AM_CFLAGS = -Wall $(ARCH_FLAGS) -std=gnu99 -I../common
AM_CCASFLAGS = $(ARCH_FLAGS)

noinst_LTLIBRARIES = libarch.la

libarch_la_LIBADD = $(top_builddir)/Transceiver52M/common/libarch_common.la

libarch_la_SOURCES = \
	convert.c \
	convert_neon.S \
	convolve.c \
	convolve_neon.S \
	scale.c \
	scale_neon.S \
	mult.c \
	mult_neon.S
