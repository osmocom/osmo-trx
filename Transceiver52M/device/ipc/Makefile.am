include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(UHD_CFLAGS)
AM_CXXFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(UHD_CFLAGS)

noinst_HEADERS = IPCDevice.h shm.h ipc_shm.h ipc_chan.h ipc_sock.h

if DEVICE_UHD
noinst_HEADERS += ../uhd/UHDDevice.h uhdwrap.h ipc-driver-test.h
endif

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = IPCDevice.cpp shm.c ipc_shm.c ipc_chan.c ipc_sock.c
libdevice_la_CPPFLAGS = $(AM_CPPFLAGS) -DIPCMAGIC
libdevice_la_LIBADD = \
	$(top_builddir)/Transceiver52M/device/common/libdevice_common.la \
	-lpthread \
	-lrt \
	$(NULL)

if DEVICE_UHD

bin_PROGRAMS = ipc-driver-test
#ipc_driver_test_SHORTNAME = drvt
ipc_driver_test_SOURCES = ipc-driver-test.c uhdwrap.cpp ../uhd/UHDDevice.cpp
ipc_driver_test_LDADD = \
	libdevice.la \
	$(COMMON_LA) \
	$(LIBOSMOCORE_LIBS) \
	$(UHD_LIBS) \
	$(NULL)
endif
