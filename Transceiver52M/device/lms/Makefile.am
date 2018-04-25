include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/..
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LMS_CFLAGS)

noinst_HEADERS = LMSDevice.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = LMSDevice.cpp
