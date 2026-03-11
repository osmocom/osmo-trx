AM_CFLAGS = -Wall -std=gnu99

noinst_LTLIBRARIES = libarch_common.la

noinst_HEADERS = \
        convolve.h \
        convert.h \
        scale.h \
        mult.h \
        $(NULL)

if ENABLE_MULTI_ARFCN
noinst_HEADERS += fft.h
endif

libarch_common_la_SOURCES = \
        convolve_base.c \
        convert_base.c \
        $(NULL)

if ENABLE_MULTI_ARFCN
libarch_common_la_SOURCES += fft.c
endif
