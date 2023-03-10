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

AM_LDFLAGS = -no-install

LDADD = \
	$(COMMON_LA) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(NULL)

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

PRBSTest_SOURCES = PRBSTest.cpp

InterthreadTest_SOURCES = InterthreadTest.cpp
InterthreadTest_LDADD = $(LDADD) -lpthread

TimevalTest_SOURCES = TimevalTest.cpp

VectorTest_SOURCES = VectorTest.cpp

LogTest_SOURCES = LogTest.cpp

MOSTLYCLEANFILES += testSource testDestination
