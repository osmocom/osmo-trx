include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/..
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(UHD_CFLAGS)

noinst_HEADERS = UHDDevice.h smpl_buf.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = UHDDevice.cpp smpl_buf.cpp
