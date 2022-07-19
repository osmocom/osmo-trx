include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS)
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS)
AM_LDFLAGS = -lpthread -lrt

noinst_HEADERS = IPCDevice.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = IPCDevice.cpp
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la
libdevice_la_CXXFLAGS = $(AM_CXXFLAGS) -DIPCMAGIC
