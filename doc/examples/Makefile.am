# all config examples must be listed here unconditionally, so that
# all of them end up in the release tarball (see OS#6349)
EXTRA_DIST = \
	osmo-trx-uhd/osmo-trx-limesdr.cfg \
	osmo-trx-uhd/osmo-trx-usrp_b200.cfg \
	osmo-trx-uhd/osmo-trx-uhd.cfg \
	osmo-trx-uhd/osmo-trx-umtrx.cfg \
	osmo-trx-lms/osmo-trx-limesdr.cfg \
	osmo-trx-lms/osmo-trx-lms.cfg \
	osmo-trx-ipc/osmo-trx-ipc.cfg \
	$(NULL)

OSMOCONF_FILES =
osmoconfdir = $(sysconfdir)/osmocom

if DEVICE_UHD
OSMOCONF_FILES += osmo-trx-uhd/osmo-trx-uhd.cfg
endif

# if DEVICE_USRP1
# TODO: no usrp1 sample file yet
# OSMOCONF_FILES += osmo-trx-usrp1/osmo-trx-usrp1.cfg
# endif

if DEVICE_LMS
OSMOCONF_FILES += osmo-trx-lms/osmo-trx-lms.cfg
endif

if DEVICE_IPC
OSMOCONF_FILES += osmo-trx-ipc/osmo-trx-ipc.cfg
endif

osmoconf_DATA = $(OSMOCONF_FILES)

CFG_FILES = find $(srcdir) -type f -name '*.cfg*' | sed -e 's,^$(srcdir),,'

dist-hook:
	for f in $$($(CFG_FILES)); do \
		j="$(distdir)/$$f" && \
		mkdir -p "$$(dirname $$j)" && \
		$(INSTALL_DATA) $(srcdir)/$$f $$j; \
	done

install-data-hook:
	for f in $$($(CFG_FILES)); do \
		j="$(DESTDIR)$(docdir)/examples/$$f" && \
		mkdir -p "$$(dirname $$j)" && \
		$(INSTALL_DATA) $(srcdir)/$$f $$j; \
	done

uninstall-hook:
	@$(PRE_UNINSTALL)
	for f in $$($(CFG_FILES)); do \
		j="$(DESTDIR)$(docdir)/examples/$$f" && \
		$(RM) $$j; \
	done
