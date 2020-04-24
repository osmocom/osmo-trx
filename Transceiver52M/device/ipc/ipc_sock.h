#ifndef IPC_SOCK_H
#define IPC_SOCK_H

#include "shm.h"
#include "ipc-driver-test.h"

int ipc_sock_send(struct msgb *msg);
int ipc_sock_accept(struct osmo_fd *bfd, unsigned int flags);
void ipc_sock_close(struct ipc_sock_state *state);

#endif // IPC_SOCK_H
