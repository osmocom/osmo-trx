if !ARCH_ARM
AM_CFLAGS = -Wall -std=gnu99 -march=native -I../common

noinst_LTLIBRARIES = libarch.la

libarch_la_SOURCES = \
	../common/convolve_base.c \
	convert.c \
	convolve.c
endif
