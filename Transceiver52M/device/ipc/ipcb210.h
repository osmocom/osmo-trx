#ifndef IPC_B210_H
#define IPC_B210_H

#include "magicwrap.h"
#include "IPCDevice.h"
#include "../uhd/UHDDevice.h"




class IPC_b210 : public IPCDevice {
    magicwrap m;
    public:
    template<typename... Args>
    IPC_b210(Args... args): IPCDevice(args...){
        //drvtest::main(0,0);
    }
    virtual ~IPC_b210() {};

	int foo(){return 32;}
//    void ipc_sock_close() override {};
};

#endif // IPC_B210_H
