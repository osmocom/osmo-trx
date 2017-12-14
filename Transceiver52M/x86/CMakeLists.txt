include_directories(../common)

set(libarch_files
	../common/convert_base.c
	../common/convolve_base.c
	convert.c
	convolve.c)

# TODO move to cmakedef
add_definitions(-DHAVE___BUILTIN_CPU_SUPPORTS)

if(HAVE_SSE4_1)
	add_definitions(-DHAVE_SSE4_1)
	set(libarch_files ${libarch_files} convert_sse_4_1.c)
endif(HAVE_SSE4_1)

if(HAVE_SSE3)
	add_definitions(-HAVE_SSE3)
	set(libarch_files ${libarch_files} convert_sse_3.c convert_sse_3.c)
endif(HAVE_SSE3)


add_library(arch ${libarch_files})

