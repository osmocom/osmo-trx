if HAVE_SYSTEMD
SYSTEMD_SERVICES =

if DEVICE_UHD
SYSTEMD_SERVICES += osmo-trx-uhd.service
endif

if DEVICE_USRP1
SYSTEMD_SERVICES += osmo-trx-usrp1.service
endif

if DEVICE_LMS
SYSTEMD_SERVICES += osmo-trx-lms.service
endif

EXTRA_DIST = $(SYSTEMD_SERVICES)
systemdsystemunit_DATA = $(SYSTEMD_SERVICES)
endif # HAVE_SYSTEMD
