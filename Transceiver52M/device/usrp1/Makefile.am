include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(USRP_CFLAGS)

noinst_HEADERS = USRPDevice.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = USRPDevice.cpp
