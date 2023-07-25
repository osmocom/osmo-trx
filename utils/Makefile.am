AM_CPPFLAGS = $(LIBOSMOCODING_CFLAGS)
AM_CFLAGS = -Wall

DIST_SUBDIRS = va-test

EXTRA_DIST = clockdump.sh matlab

noinst_PROGRAMS = osmo-prbs-tool

osmo_prbs_tool_SOURCES = prbs-tool.c
osmo_prbs_tool_LDADD = $(LIBOSMOCODING_LIBS)
