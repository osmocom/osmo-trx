#pragma once

#include <radioVector.h>
#include <signalVector.h>

enum class trx_mode {
	TRX_MODE_OFF,
	TRX_MODE_BTS,
	TRX_MODE_MS_ACQUIRE,
	TRX_MODE_MS_TRACK,
};

enum class ChannelCombination {
	FILL, ///< Channel is transmitted, but unused
	I, ///< TCH/FS
	II, ///< TCH/HS, idle every other slot
	III, ///< TCH/HS
	IV, ///< FCCH+SCH+CCCH+BCCH, uplink RACH
	V, ///< FCCH+SCH+CCCH+BCCH+SDCCH/4+SACCH/4, uplink RACH+SDCCH/4
	VI, ///< CCCH+BCCH, uplink RACH
	VII, ///< SDCCH/8 + SACCH/8
	VIII, ///< TCH/F + FACCH/F + SACCH/M
	IX, ///< TCH/F + SACCH/M
	X, ///< TCH/FD + SACCH/MD
	XI, ///< PBCCH+PCCCH+PDTCH+PACCH+PTCCH
	XII, ///< PCCCH+PDTCH+PACCH+PTCCH
	XIII, ///< PDTCH+PACCH+PTCCH
	NONE_INACTIVE, ///< Channel is inactive, default
	LOOPBACK ///< similar go VII, used in loopback testing
};

struct ms_TransceiverState {
	ms_TransceiverState() : mFreqOffsets(10), mode(trx_mode::TRX_MODE_OFF)
	{
		for (int i = 0; i < 8; i++) {
			chanType[i] = ChannelCombination::NONE_INACTIVE;
			fillerModulus[i] = 26;

			for (int n = 0; n < 102; n++)
				fillerTable[n][i] = nullptr;
		}
	}

	~ms_TransceiverState()
	{
		for (int i = 0; i < 8; i++) {
			for (int n = 0; n < 102; n++)
				delete fillerTable[n][i];
		}
	}

	void setModulus(size_t timeslot)
	{
		switch (chanType[timeslot]) {
		case ChannelCombination::NONE_INACTIVE:
		case ChannelCombination::I:
		case ChannelCombination::II:
		case ChannelCombination::III:
		case ChannelCombination::FILL:
			fillerModulus[timeslot] = 26;
			break;
		case ChannelCombination::IV:
		case ChannelCombination::VI:
		case ChannelCombination::V:
			fillerModulus[timeslot] = 51;
			break;
			//case V:
		case ChannelCombination::VII:
			fillerModulus[timeslot] = 102;
			break;
		case ChannelCombination::XIII:
			fillerModulus[timeslot] = 52;
			break;
		default:
			break;
		}
	}

	CorrType expectedCorrType(GSM::Time currTime, unsigned long long *mRxSlotMask)
	{
		unsigned burstTN = currTime.TN();
		unsigned burstFN = currTime.FN();

		if (mode == trx_mode::TRX_MODE_MS_TRACK) {
			/* 102 modulus case currently unhandled */
			if (fillerModulus[burstTN] > 52)
				return OFF;

			int modFN = burstFN % fillerModulus[burstTN];
			unsigned long long reg = (unsigned long long)1 << modFN;
			if (reg & mRxSlotMask[burstTN])
				return TSC;
			else
				return OFF;
		}

		switch (chanType[burstTN]) {
		case ChannelCombination::NONE_INACTIVE:
			return OFF;
			break;
		case ChannelCombination::FILL:
			return IDLE;
			break;
		case ChannelCombination::I:
			return TSC;
			/*if (burstFN % 26 == 25) 
      return IDLE;
    else
      return TSC;*/
			break;
		case ChannelCombination::II:
			return TSC;
			break;
		case ChannelCombination::III:
			return TSC;
			break;
		case ChannelCombination::IV:
		case ChannelCombination::VI:
			return RACH;
			break;
		case ChannelCombination::V: {
			int mod51 = burstFN % 51;
			if ((mod51 <= 36) && (mod51 >= 14))
				return RACH;
			else if ((mod51 == 4) || (mod51 == 5))
				return RACH;
			else if ((mod51 == 45) || (mod51 == 46))
				return RACH;
			else
				return TSC;
			break;
		}
		case ChannelCombination::VII:
			if ((burstFN % 51 <= 14) && (burstFN % 51 >= 12))
				return IDLE;
			else
				return TSC;
			break;
		case ChannelCombination::XIII: {
			int mod52 = burstFN % 52;
			if ((mod52 == 12) || (mod52 == 38))
				return RACH;
			else if ((mod52 == 25) || (mod52 == 51))
				return IDLE;
			else
				return TSC;
			break;
		}
		case ChannelCombination::LOOPBACK:
			if ((burstFN % 51 <= 50) && (burstFN % 51 >= 48))
				return IDLE;
			else
				return TSC;
			break;
		default:
			return OFF;
			break;
		}
	}

	/* Initialize a multiframe slot in the filler table */
	void init(size_t slot, signalVector *burst, bool fill);

	ChannelCombination chanType[8];

	/* The filler table */
	signalVector *fillerTable[102][8];
	int fillerModulus[8];

	/* Received noise energy levels */
	avgVector mFreqOffsets;

	/* Transceiver mode */
	trx_mode mode;
};