include $(top_srcdir)/Makefile.common

AM_CFLAGS = -Wall -I$(top_srcdir)/Transceiver52M -I$(top_srcdir)/Transceiver52M/arch/common $(STD_DEFINES_AND_INCLUDES) -g

EXTRA_DIST = convolve_test.ok

noinst_PROGRAMS = \
	convolve_test

convolve_test_SOURCES = convolve_test.c
convolve_test_CFLAGS = $(AM_CFLAGS)
convolve_test_LDADD = $(COMMON_LA) $(ARCH_LA)
if HAVE_SSE3
convolve_test_CFLAGS += $(SIMD_FLAGS)
endif
if HAVE_SSE4_1
convolve_test_CFLAGS += $(SIMD_FLAGS)
endif

if DEVICE_LMS
noinst_PROGRAMS += LMSDeviceTest
LMSDeviceTest_SOURCES = LMSDeviceTest.cpp
LMSDeviceTest_LDFLAGS = $(LIBOSMOCORE_LIBS) $(LMS_LIBS)
LMSDeviceTest_LDADD = \
	$(top_builddir)/Transceiver52M/device/lms/libdevice.la \
	$(COMMON_LA) \
	$(LMS_LIBS)
LMSDeviceTest_CPPFLAGS = $(AM_CPPFLAGS) $(LMS_CFLAGS)
endif
