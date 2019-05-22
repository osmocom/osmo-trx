EXTRA_DIST = osmotrx-usermanual.adoc \
    osmotrx-usermanual-docinfo.xml \
    osmotrx-vty-reference.xml \
    chapters \
    vty

if BUILD_MANUALS
  ASCIIDOC = osmotrx-usermanual.adoc
  ASCIIDOC_DEPS = $(srcdir)/chapters/*.adoc
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.asciidoc.inc

  VTY_REFERENCE = osmotrx-vty-reference.xml
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.vty-reference.inc

  OSMO_REPOSITORY = osmo-trx
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.common.inc
endif
