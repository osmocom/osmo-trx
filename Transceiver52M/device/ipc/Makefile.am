include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common
AM_CFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)
AM_LDFLAGS = -lpthread -lrt

noinst_HEADERS = IPCDevice.h shm.h ipcb210.h ../uhd/UHDDevice.h uhdwrap.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = ipc-driver-test.c ipcb210.cpp IPCDevice.cpp shm.c ../uhd/UHDDevice.cpp uhdwrap.cpp ipc_shm.c ipc_chan.c ipc_sock.c 
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la
libdevice_la_CXXFLAGS = $(AM_CXXFLAGS) $(UHD_CFLAGS) -DIPCMAGIC

bin_PROGRAMS = ipc-driver-test
#ipc_driver_test_SHORTNAME = drvt
ipc_driver_test_SOURCES = ipc-driver-test.c uhdwrap.cpp ipc_shm.c ipc_chan.c ipc_sock.c ../uhd/UHDDevice.cpp
ipc_driver_test_LDADD = \
        shm.lo \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)
ipc_driver_test_CXXFLAGS = $(AM_CXXFLAGS) $(UHD_CFLAGS)
ipc_driver_test_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS)
ipc_driver_test_CFLAGS  = $(AM_CFLAGS) $(UHD_CFLAGS)
ipc_driver_test_LDFLAGS  = $(AM_LDFLAGS) $(UHD_LIBS)
ipc_driver_test_LDADD += $(top_builddir)/Transceiver52M/device/common/libdevice_common.la $(top_builddir)/CommonLibs/libcommon.la

