include $(top_srcdir)/Makefile.common

SUBDIRS = common

if DEVICE_IPC
SUBDIRS += ipc ipc2
endif

if DEVICE_USRP1
SUBDIRS += usrp1
endif

if DEVICE_UHD
SUBDIRS += uhd
endif

if DEVICE_LMS
SUBDIRS += lms
endif

if DEVICE_BLADE
SUBDIRS += bladerf
endif
