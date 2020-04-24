#ifndef IPC_B210_H
#define IPC_B210_H

#include <thread>
//#include "magicwrap.h"
#include "IPCDevice.h"
//#include "../uhd/UHDDevice.h"

class IPC_b210 : public IPCDevice {
	//	std::thread *t;
	//	bool flush_recv(size_t num_pkts) override;

    public:
	template <typename... Args> IPC_b210(Args... args);
	virtual ~IPC_b210();

	//    void ipc_sock_close() override {};
#if 0
	int readSamples(std::vector<short *> &bufs, int len, bool *overrun, TIMESTAMP timestamp,
			bool *underrun) override;
    TIMESTAMP initialWriteTimestamp() override;
	TIMESTAMP initialReadTimestamp() override;
    bool start() override;
#endif

#if 0
	int writeSamples(std::vector<short *> &bufs, int len, bool *underrun, unsigned long long timestamp) override;
	    bool updateAlignment(TIMESTAMP timestamp) override;
#endif
	//	double setTxGain(double dB, size_t chan) override;
	//	double setRxGain(double dB, size_t chan) override;

	//	bool stop() override;
};

#endif // IPC_B210_H
