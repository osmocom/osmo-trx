#ifndef RADIO_INTERFACE_MULTI_H
#define RADIO_INTERFACE_MULTI_H

#include "radioInterface.h"
#include "Channelizer.h"
#include "Synthesis.h"

class RadioInterfaceMulti : public RadioInterface {
public:
	RadioInterfaceMulti(RadioDevice* radio, size_t tx_sps,
			    size_t rx_sps, size_t chans = 1);
	virtual ~RadioInterfaceMulti();

	bool init(int type);
	void close();

	bool tuneTx(double freq, size_t chan);
	bool tuneRx(double freq, size_t chan);
	virtual double setRxGain(double dB, size_t chan);
	virtual double rssiOffset(size_t chan = 0);

private:
	bool pushBuffer();
	int pullBuffer();
	bool verify_arfcn_consistency(double freq, size_t chan, bool tx);
	virtual int setPowerAttenuation(int atten, size_t chan = 0);

	signalVector *outerSendBuffer;
	signalVector *outerRecvBuffer;
	std::vector<signalVector *> history;
	std::vector<bool> active;
	std::vector<struct freq_cfg_state> rx_freq_state;
	std::vector<struct freq_cfg_state> tx_freq_state;

	Resampler *dnsampler;
	Resampler *upsampler;
	Channelizer *channelizer;
	Synthesis *synthesis;
};

#endif /* RADIO_INTERFACE_MULTI_H */
