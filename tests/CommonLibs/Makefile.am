include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = \
	-I$(top_srcdir)/CommonLibs \
	$(STD_DEFINES_AND_INCLUDES) \
	$(NULL)

AM_CXXFLAGS = \
	-Wall -g \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(NULL)

AM_LDFLAGS = $(LIBOSMOCORE_LIBS) $(LIBOSMOCTRL_LIBS) $(LIBOSMOVTY_LIBS)

EXTRA_DIST = BitVectorTest.ok \
             PRBSTest.ok \
             InterthreadTest.ok \
             TimevalTest.ok \
             VectorTest.ok \
             LogTest.ok \
             LogTest.err

check_PROGRAMS = \
	BitVectorTest \
	PRBSTest \
	InterthreadTest \
	TimevalTest \
	VectorTest \
	LogTest

BitVectorTest_SOURCES = BitVectorTest.cpp
BitVectorTest_LDADD = $(COMMON_LA)

PRBSTest_SOURCES = PRBSTest.cpp

InterthreadTest_SOURCES = InterthreadTest.cpp
InterthreadTest_LDADD = $(COMMON_LA)
InterthreadTest_LDFLAGS = -lpthread $(AM_LDFLAGS)

TimevalTest_SOURCES = TimevalTest.cpp
TimevalTest_LDADD = $(COMMON_LA)

VectorTest_SOURCES = VectorTest.cpp
VectorTest_LDADD = $(COMMON_LA)

LogTest_SOURCES = LogTest.cpp
LogTest_LDADD = $(COMMON_LA)

MOSTLYCLEANFILES += testSource testDestination
