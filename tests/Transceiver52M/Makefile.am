include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = \
	-I$(top_srcdir)/Transceiver52M \
	-I$(top_srcdir)/Transceiver52M/arch/common \
	$(STD_DEFINES_AND_INCLUDES) \
	$(NULL)

AM_CFLAGS = -Wall -g
AM_LDFLAGS = -no-install

EXTRA_DIST = convolve_test.ok convolve_test_golden.h

check_PROGRAMS = \
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
check_PROGRAMS += LMSDeviceTest
LMSDeviceTest_SOURCES = LMSDeviceTest.cpp
LMSDeviceTest_LDADD = \
	$(top_builddir)/Transceiver52M/device/lms/libdevice.la \
	$(LIBOSMOCORE_LIBS) \
	$(COMMON_LA) \
	$(LMS_LIBS) \
	$(NULL)
LMSDeviceTest_CPPFLAGS = $(AM_CPPFLAGS) $(LMS_CFLAGS)
endif
