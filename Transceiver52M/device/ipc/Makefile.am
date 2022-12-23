include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS)
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS)
AM_LDFLAGS = -lpthread -lrt

noinst_HEADERS = IPCDevice.h shm.h ipc_shm.h ipc_chan.h ipc_sock.h

if DEVICE_UHD
noinst_HEADERS += ../uhd/UHDDevice.h uhdwrap.h ipc-driver-test.h
endif

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = IPCDevice.cpp shm.c ipc_shm.c ipc_chan.c ipc_sock.c
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la
libdevice_la_CXXFLAGS = $(AM_CXXFLAGS) -DIPCMAGIC

if DEVICE_UHD

bin_PROGRAMS = ipc-driver-test
#ipc_driver_test_SHORTNAME = drvt
ipc_driver_test_SOURCES = ipc-driver-test.c uhdwrap.cpp ipc_shm.c ipc_chan.c ipc_sock.c ../uhd/UHDDevice.cpp
ipc_driver_test_LDADD = \
        shm.lo \
	$(top_builddir)/Transceiver52M/device/common/libdevice_common.la \
	$(COMMON_LA)
	$(LIBOSMOCORE_LIBS) \
	$(NULL)
ipc_driver_test_CXXFLAGS = $(AM_CXXFLAGS) $(UHD_CFLAGS)
ipc_driver_test_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS)
ipc_driver_test_CFLAGS  = $(AM_CFLAGS) $(UHD_CFLAGS)
ipc_driver_test_LDFLAGS  = $(AM_LDFLAGS) $(UHD_LIBS)
endif
