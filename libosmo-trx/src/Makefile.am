# This is _NOT_ the library release version, it's an API version.
# Please read chapter "Library interface versions" of the libtool
# documentation before making any modification
LIBVERSION = 0:0:0

AM_CPPFLAGS = \
	-I$(top_srcdir)/libosmo-trx/include \
	-I$(top_builddir)/libosmo-trx/include \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(NULL)

lib_LTLIBRARIES = libosmo-trx.la

libosmo_trx_la_SOURCES = \
	trxc.c \
	trxd.c \
	$(NULL)

libosmo_trx_la_LDFLAGS = \
	-version-info $(LIBVERSION) \
	-no-undefined \
	-export-symbols-regex '^osmo_' \
	$(NULL)

libosmo_trx_la_LIBADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)
