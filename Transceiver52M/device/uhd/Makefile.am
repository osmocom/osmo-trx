include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/..
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(UHD_CFLAGS)

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = UHDDevice.cpp
