EXTRA_DIST = osmotrx-usermanual.adoc \
    osmotrx-usermanual-docinfo.xml \
    chapters \
    vty

if BUILD_MANUALS
  ASCIIDOC = osmotrx-usermanual.adoc
  ASCIIDOC_DEPS = $(srcdir)/chapters/*.adoc
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.asciidoc.inc

  VARIANTS = $(NULL)

if DEVICE_UHD
  VARIANTS += uhd
endif
if DEVICE_USRP1
  VARIANTS += usrp1
endif
if DEVICE_LMS
  VARIANTS += lms
endif
if DEVICE_IPC
  VARIANTS += ipc
endif

  # This is a significantly modified, multi-target adopted copy of
  # $(OSMO_GSM_MANUALS_DIR)/build/Makefile.vty-reference.inc
  include $(srcdir)/vty/Makefile.vty-reference.inc

  OSMO_REPOSITORY = osmo-trx
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.common.inc
endif
