include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	-Iinclude \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOCODING_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	-Iinclude \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOCODING_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(NULL)


AM_CPPFLAGS += $(STD_DEFINES_AND_INCLUDES)
AM_CXXFLAGS = -Wall -O3 -g -ldl -lpthread -Iinclude

noinst_LTLIBRARIES = libtrxcon.la 

libtrxcon_la_SOURCES = \
	src/sched_lchan_common.c \
	src/sched_lchan_pdtch.c \
	src/sched_lchan_desc.c \
	src/sched_lchan_xcch.c \
	src/sched_lchan_tchf.c \
	src/sched_lchan_tchh.c \
	src/sched_lchan_rach.c \
	src/sched_lchan_sch.c \
	src/sched_mframe.c \
	src/sched_clck.c \
	src/sched_prim.c \
	src/sched_trx.c \
	src/l1ctl_server.c \
	src/l1ctl.c \
	src/trx_if.c \
	src/logging.c \
	src/trxcon_fsm.c \
	src/trxcon.c \
	$(NULL)

libtrxcon_la_LIBADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCODING_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)
