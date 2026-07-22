SUBDIRS = \
	include \
	src \
	$(NULL)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libosmotrx.pc

if HAVE_DOXYGEN

# Own doc package (libosmo-trx-doc), not $(top_srcdir)'s docdir/osmo-trx.
# Automake appends its own "htmldir = ${docdir}" default further down in
# the generated Makefile, which would silently win over a plain assignment
# here; "override" is required to make our value stick.
override htmldir = $(datarootdir)/doc/libosmo-trx

html_DATA = $(builddir)/doc/html.tar

$(html_DATA): $(builddir)/doc/html/index.html
	cd $(builddir)/doc && tar cf html.tar html

$(builddir)/doc/html/index.html: $(srcdir)/src/*.c \
		$(srcdir)/include/osmocom/trx/*.h \
		Doxyfile.trx
	rm -rf $(builddir)/doc/html; mkdir -p $(builddir)/doc
	$(DOXYGEN) Doxyfile.trx

install-data-hook:
	cd $(DESTDIR)$(htmldir) && tar xf html.tar && rm -f html.tar

uninstall-hook:
	cd $(DESTDIR)$(htmldir) && rm -rf html

DX_CLEAN = \
	doc/html/search/* \
	doc/html/* \
	doc/doxygen_sqlite3.db \
	doc/html.tar \
	$(NULL)

endif

MOSTLYCLEANFILES = $(DX_CLEAN)
