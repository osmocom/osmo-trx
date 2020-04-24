#ifndef IPC_CHAN_H
#define IPC_CHAN_H

#include "shm.h"
#include "ipc-driver-test.h"

int ipc_chan_sock_send(struct msgb *msg, uint8_t chan_nr);
int ipc_chan_sock_accept(struct osmo_fd *bfd, unsigned int flags);

#endif // IPC_CHAN_H
