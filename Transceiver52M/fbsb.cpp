#include <atomic>
#include "radioInterface.h"
#include "Interthread.h"
#include "GSMCommon.h"
//#include "Sockets.h"

#include <sys/types.h>
#include <sys/socket.h>


static const auto tssize = 156.25;
static const auto framee_to_read = 12;
static const auto ts_to_read = framee_to_read * 8;
static const int fbsb_chunk_len = ts_to_read * 2 * tssize; // contiguous fcch+sch guaranteed
static const auto corr_offset = 0;

struct fbsb_par {

    enum class fbsb_state {
	IDLE,
	INIT,
        ACQ,
        WAIT,
	ACQ_COMPL,
        DONE
    };

    int pos;
    int time_idx;
    std::atomic<fbsb_state> s;
    GSM::Time rcvClock[ts_to_read];
    complex fbsb_buf[fbsb_chunk_len+corr_offset];
    complex conjbuf[fbsb_chunk_len+corr_offset];
    complex avgbuf[fbsb_chunk_len+corr_offset];
    void addclk(GSM::Time c) { rcvClock[time_idx] = c; time_idx++; return;}
public:
    fbsb_par() : s(fbsb_state::IDLE) {
	reset();
    };
    bool done() {return !(time_idx < ts_to_read);}
    void take(void* addr, int num_cplx_sps, GSM::Time c) {
        memcpy(fbsb_buf+pos+corr_offset,addr, num_cplx_sps * sizeof(complex));
	pos += num_cplx_sps;
	assert(pos < fbsb_chunk_len);
	rcvClock[time_idx] = c;
	time_idx++;
    }
    void reset() {pos = 0;
		  time_idx = 0;
//		s = fbsb_state::IDLE;
		  memset(fbsb_buf, 0, sizeof(fbsb_buf));
		  memset(conjbuf, 0, sizeof(conjbuf));
		  memset(avgbuf, 0, sizeof(avgbuf));
		 }
    int sz () {return fbsb_chunk_len;}
    int off () {return corr_offset;}

    // from osmotrx sigproc
    complex fastPeakDetect(complex* rxBurst, float* index, int N)
    {
	    float val, max = 0.0f;
	    complex amp;
	    int _index = -1;

	    for (size_t i = 0; i < N; i++) {
		    val = rxBurst[i].abs();
		    if (val > max) {
			    max = val;
			    _index = i;
			    amp = rxBurst[i];
		    }
	    }

	    if (index)
		    *index = (float)_index;

	    return amp;
    }

    void conj_with_lag(complex* in, complex* out, int lag, int offset, int N) {
	    int total_offset = offset + lag;
	    for (int s = 0; s < N; s++)
		    out[s] = in[s + total_offset] * in[s + total_offset - lag].conj();
    }

    auto ac_sum_with_lag(complex* in, int lag, int offset, int N) {
	    complex v(0,0);
	    auto total_offset = offset + lag;
	    for (auto s = 0; s < N; s++)
		    v += in[s + total_offset] * in[s + total_offset - lag].conj();
	    return atan2(v.imag(), v.real());
    }


    bool running_avg_opt(complex* in, complex* out, int avg_window, int val_to_find, int* idx, int N) {
	    bool found = false;
	    complex avg0 = 0;
	    complex scale(avg_window, avg_window);
	    for (auto i = 0; i < avg_window; i++)
		    avg0 += in[i];
	    out[0] = avg0 / scale;

	    //skip first
	    for (auto i = 1; i < N - avg_window; i++) {
		    avg0 += in[i-1] - in[i+avg_window];
		    auto tmp = avg0 / scale;
		    out[i] = tmp;
		    if (!found && tmp.abs() > val_to_find) {
			    found = !found;
			    *idx = i;
			    return true;
		    }
	    }
	    return false;
    }

    int fcch(complex* amp, int* found_idx, bool dump) {
	conj_with_lag(fbsb_buf, conjbuf, 3, off(), sz());

	running_avg_opt(conjbuf, avgbuf, 48, 1.5e6, found_idx, sz()-off());
	float pos;
	auto r = fastPeakDetect(avgbuf, &pos, sz()-off());
	*found_idx = *found_idx+off();
	std::cerr << "fcch found at " << pos << " amp: " << r.abs() << std::endl;
	*amp = r;

	if(dump) {
		{
		    auto f = fopen("inbuf.cfile", "wb");
		    fwrite(fbsb_buf, sz(), 1, f);
		    fclose(f);
		}
		{
		    auto f = fopen("conjbuf.cfile", "wb");
		    fwrite(conjbuf, sz(), 1, f);
		    fclose(f);
		}
		{
		    auto f = fopen("schfcch.cfile", "wb");
		    fwrite(avgbuf, sz(), 1, f);
		    fclose(f);
		}
		exit(0);
	    }
	return pos;
    }
};

