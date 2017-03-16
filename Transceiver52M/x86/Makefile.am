if !ARCH_ARM
AM_CFLAGS = -Wall -std=gnu99 $(SIMD_FLAGS) -I${srcdir}/../common

noinst_LTLIBRARIES = libarch.la

libarch_la_SOURCES = \
	../common/convolve_base.c \
	../common/convert_base.c \
	convert.c \
	convolve.c
endif
