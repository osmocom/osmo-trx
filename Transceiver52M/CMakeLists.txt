if(NOT TRANS_FULL_VERSION)
    add_definitions(-DNO_RESAMPLER)
    add_definitions(-DNO_MULTIARFCN)
endif()


add_subdirectory(x86)
include_directories(common)
include_directories(".")

set(COMMON_FILES
	radioInterface.cpp
	radioVector.cpp
	radioClock.cpp
	radioBuffer.cpp
	sigProcLib.cpp
	signalVector.cpp
	Transceiver.cpp)

set(libtransceiver_files
	Resampler.cpp
	${COMMON_FILES})

if(TRANS_FULL_VERSION)
	set(libtransceiver_files
		${libtransceiver_files}
		radioInterfaceResamp.cpp
		radioInterfaceMulti.cpp
		ChannelizerBase.cpp
		Channelizer.cpp
		Synthesis.cpp
		common/fft.c
		radioInterfaceDiversity.cpp)
endif(TRANS_FULL_VERSION)

set(noinst_HEADERS
	Complex.h
	radioInterface.h
	radioVector.h
	radioClock.h
	radioDevice.h
	radioBuffer.h
	sigProcLib.h
	signalVector.h
	Transceiver.h
	USRPDevice.h
	Resampler.h
	ChannelizerBase.h
	Channelizer.h
	Synthesis.h
	common/convolve.h
	common/convert.h
	common/scale.h
	common/mult.h
	common/fft.h)

add_library(transceiver ${libtransceiver_files})


set(DEVICE        XTRXDevice.cpp)
set(DEVICE_LIBS   ${XTRX_LIBRARIES})
set(DEVICE_INC    ${XTRX_INCLUDES})

include_directories(${DEVICE_INC})
add_executable(osmo-trx osmo-trx.cpp ${DEVICE})
target_link_libraries(osmo-trx transceiver arch GSM common ${sqlite3} ${FFTW_LIBRARIES} ${DEVICE_LIBS} pthread dl)

