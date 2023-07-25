include $(top_srcdir)/Makefile.common

noinst_PROGRAMS = osmo-burst-gen

osmo_burst_gen_SOURCES = burst-gen.cpp \
	${top_srcdir}/Transceiver52M/grgsm_vitac/grgsm_vitac.cpp \
	${top_srcdir}/Transceiver52M/grgsm_vitac/viterbi_detector.cc
osmo_burst_gen_LDADD = \
	${top_srcdir}/Transceiver52M/libtransceiver_common.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA)
osmo_burst_gen_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) \
	-I${top_srcdir}/Transceiver52M/arch/common \
	-I${top_srcdir}/Transceiver52M/device/common \
	-I${top_srcdir}/Transceiver52M

