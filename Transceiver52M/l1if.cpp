#include <mutex>
#include <queue>
#include <deque>
#include <condition_variable>
#include <iostream>

extern "C" {

#include <unistd.h>
#include <sys/eventfd.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
}
#include "l1if.h"

using namespace std;
using namespace std::chrono_literals;

template<typename Data>
class spsc_q{

    std::queue<Data> m_q;
    std::mutex m_mtx;
    std::condition_variable m_cond;
    bool killme;

public:
    spsc_q() : killme{ false } { }

    void push(Data i){
        std::unique_lock<std::mutex> lock(m_mtx);
        m_q.push(i);
        m_cond.notify_one();
    }

    Data pop(){
        std::unique_lock<std::mutex> lock(m_mtx);
        m_cond.wait_for(lock, 100ms, [&](){ return !m_q.empty() || killme; });

        if (killme || m_q.empty()){
                return {};
            }

        Data x = m_q.front();
        m_q.pop();

        return x;
    }

    void stop(){
        killme = true;
        m_cond.notify_all();
    }

    auto sz() { return m_q.size(); }
};


/*
 * trxif_from_trx_c <-> push_c
 * trxif_to_trx_c <-> pop_c
 * trxif_from_trx_d <-> push_d
 * trxif_to_trx_d <-> pop_d
 * ...
 *
 *
 */
class trxl1if {
public:
    spsc_q<TRX_C*> c_to_trx;
    spsc_q<TRX_C*> c_from_trx;

    spsc_q<trxd_to_trx*> d_to_trx;
    spsc_q<trxd_from_trx*> d_from_trx;

    struct osmo_fd g_event_ofd_C;
    struct osmo_fd g_event_ofd_D;
};

trxl1if trxif;

void push_c(TRX_C* i) {
	uint64_t one = 1;
	int rc;
	trxif.c_from_trx.push(i);
	// std::clog << trxif.c_from_trx.sz() << std::endl;
	rc = ::write(trxif.g_event_ofd_C.fd, &one, sizeof(one));
	return;
};
TRX_C* pop_c() {
	return trxif.c_to_trx.pop();
};
void push_d(trxd_from_trx* i) {
	uint64_t one = 1;
	int rc;
	trxif.d_from_trx.push(i);
	rc = ::write(trxif.g_event_ofd_D.fd, &one, sizeof(one));
	return;
};
trxd_to_trx* pop_d() {
	return trxif.d_to_trx.pop();
};

extern "C" {
char* trxif_from_trx_c() {
    uint64_t one = 1;
    ::read(trxif.g_event_ofd_C.fd, &one, sizeof(one));
    return (char*)trxif.c_from_trx.pop();
}
void trxif_to_trx_c(char* msg) {
    trxif.c_to_trx.push((TRX_C*)msg);
}
trxd_from_trx* trxif_from_trx_d() {
    uint64_t one = 1;
    ::read(trxif.g_event_ofd_D.fd, &one, sizeof(one));
    return trxif.d_from_trx.pop();
}
void trxif_to_trx_d(trxd_to_trx* msg) {
    trxif.d_to_trx.push(msg);
}
struct osmo_fd* get_c_fd() { return &trxif.g_event_ofd_C;}
struct osmo_fd* get_d_fd() { return &trxif.g_event_ofd_D;}
}
