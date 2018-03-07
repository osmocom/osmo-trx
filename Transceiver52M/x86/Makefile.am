AM_CFLAGS = -Wall -std=gnu99 -I${srcdir}/../common

noinst_LTLIBRARIES = libarch.la
noinst_LTLIBRARIES += libarch_sse_3.la
noinst_LTLIBRARIES += libarch_sse_4_1.la

libarch_la_LIBADD = $(top_builddir)/Transceiver52M/common/libarch_common.la

# SSE 3 specific code
if HAVE_SSE3
libarch_sse_3_la_SOURCES = \
	convert_sse_3.c \
	convolve_sse_3.c
libarch_sse_3_la_CFLAGS = $(AM_CFLAGS) -msse3
libarch_la_LIBADD += libarch_sse_3.la
endif

# SSE 4.1 specific code
if HAVE_SSE4_1
libarch_sse_4_1_la_SOURCES = \
	convert_sse_4_1.c
libarch_sse_4_1_la_CFLAGS = $(AM_CFLAGS) -msse4.1
libarch_la_LIBADD += libarch_sse_4_1.la
endif

libarch_la_SOURCES = \
	convert.c \
	convolve.c
