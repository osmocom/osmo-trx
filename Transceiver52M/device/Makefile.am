include $(top_srcdir)/Makefile.common

noinst_HEADERS = radioDevice.h smpl_buf.h

SUBDIRS =

if DEVICE_USRP1
SUBDIRS += usrp1
endif

if DEVICE_UHD
SUBDIRS += uhd
endif

if DEVICE_LMS
SUBDIRS += lms
endif
