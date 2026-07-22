AM_CPPFLAGS = \
	-I$(top_srcdir)/libosmo-trx/include \
	-I$(top_builddir)/libosmo-trx/include \
	$(NULL)

AM_CFLAGS = \
	-Wall -g \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(NULL)

# Link statically, so that the tests can also call symbols hidden
# from the public library ABI (-export-symbols-regex).
AM_LDFLAGS = -static

LDADD = \
	$(top_builddir)/libosmo-trx/src/libosmotrx.la \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)

EXTRA_DIST = \
	trxc_test.ok \
	trxc_client_test.ok \
	trxc_client_test.err \
	trxd_test.ok \
	$(NULL)

check_PROGRAMS = \
	trxc_test \
	trxc_client_test \
	trxd_test \
	$(NULL)

trxc_test_SOURCES = trxc_test.c

trxc_client_test_SOURCES = trxc_client_test.c

trxd_test_SOURCES = trxd_test.c
