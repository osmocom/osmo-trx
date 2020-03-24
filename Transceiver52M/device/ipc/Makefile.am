include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)
AM_LDFLAGS = -lpthread -lrt

noinst_HEADERS = IPCDevice.h shm.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = IPCDevice.cpp shm.c
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la


bin_PROGRAMS = ipc-driver-test
ipc_driver_test_SOURCES = ipc-driver-test.c
ipc_driver_test_LDADD = \
        shm.lo \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)
ipc_driver_test_CPPFLAGS  = $(AM_CPPFLAGS)
ipc_driver_test_CFLAGS  = $(AM_CFLAGS)
ipc_driver_test_LDFLAGS  = $(AM_LDFLAGS)
