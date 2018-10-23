EXTRA_DIST = \
  osmo-trx-lms.service \
  osmo-trx-uhd.service \
  osmo-trx-usrp1.service

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

if DEVICE_XTRX
SYSTEMD_SERVICES += osmo-trx-xtrx.service
endif

systemdsystemunit_DATA = $(SYSTEMD_SERVICES)
endif # HAVE_SYSTEMD
