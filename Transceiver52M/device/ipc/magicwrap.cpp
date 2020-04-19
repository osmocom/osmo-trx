//has to come first outside of NS, or wrapping of c headers fails due to wrong order
#include <cstdlib>
#include <chrono>
#include "magicwrap.h"

namespace drvtest
{
extern "C" {
#define IPCMAGIC
#include "ipc-driver-test.c"
}
} // namespace drvtest

char *argv[] = { { "magic" } };
magicwrap::magicwrap()
{
	t = new std::thread([] { drvtest::magicmain(1, argv); });
	// give the thread some time to start and set up
	std::this_thread::sleep_for(std::chrono::seconds(1));
}
