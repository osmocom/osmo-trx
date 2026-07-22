AM_CPPFLAGS = \
	-Wall \
	-I$(top_srcdir)/proxy/include \
	-I$(top_srcdir)/libosmo-trx/include \
	-I$(top_builddir)/libosmo-trx/include \
	$(NULL)

AM_CFLAGS = \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(NULL)

bin_PROGRAMS = osmo-trx-proxy

osmo_trx_proxy_SOURCES = \
	main.c \
	proxy.c \
	vty.c \
	trx.c \
	logging.c \
	$(NULL)

osmo_trx_proxy_LDADD = \
	$(top_builddir)/libosmo-trx/src/libosmo-trx.la \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)
