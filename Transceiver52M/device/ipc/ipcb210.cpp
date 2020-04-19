#include "ipcb210.h"

#include "trx_vty.h"
#include "Logger.h"
#include "Threads.h"
#include "Utils.h"
#include "IPCDevice.h"




RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "IPC Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "IPC doesn't support lo_offset";
		return NULL;
	}
	return new IPC_b210(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
