/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: 0BSD
*
* Permission to use, copy, modify, and/or distribute this software for any purpose
* with or without fee is hereby granted.THE SOFTWARE IS PROVIDED "AS IS" AND THE
* AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
* BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
* CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
* USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#ifndef IPC_CHAN_H
#define IPC_CHAN_H

#include "shm.h"
#include "ipc-driver-test.h"

int ipc_chan_sock_send(struct msgb *msg, uint8_t chan_nr);
int ipc_chan_sock_accept(struct osmo_fd *bfd, unsigned int flags);

#endif // IPC_CHAN_H
