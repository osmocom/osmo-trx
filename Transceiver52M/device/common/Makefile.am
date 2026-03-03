include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES)
AM_CXXFLAGS = -lpthread $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCORE_CFLAGS)


noinst_HEADERS = radioDevice.h smpl_buf.h bandmanager.h

noinst_LTLIBRARIES = libdevice_common.la

libdevice_common_la_SOURCES = \
	smpl_buf.cpp

libdevice_common_la_LIBADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)
