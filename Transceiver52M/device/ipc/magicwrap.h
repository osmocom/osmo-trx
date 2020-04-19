#ifndef MAGICWRAP_H
#define MAGICWRAP_H

#include <thread>


class magicwrap
{
    std::thread* t;
public:
    magicwrap();
};

#endif // MAGICWRAP_H
