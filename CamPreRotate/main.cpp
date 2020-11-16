/*
    do:
        git submodule update --init --recursive

    for submodules and submodules of submodules...
*/

#include <windows.h>

#include <quat.h>
#include <vrpn_Connection.h>

#define WS_VER_MAJOR 2
#define WS_VER_MINOR 2
#pragma comment(lib, "ws2_32.lib")

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

static volatile int done = 0;

// install a signal handler to shut down the devices
// On Windows, the signal handler is run in a different thread from
// the main application.  We don't want to go destroying things in
// here while they are being used there, so we set a flag telling the
// main program it is time to exit.
#if defined(_WIN32) && !defined(__CYGWIN__)
/**
* Handle exiting cleanly when we get ^C or other signals.
*/
BOOL WINAPI handleConsoleSignalsWin(DWORD signaltype)
{
    switch (signaltype) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        done = 1;
        return TRUE;
        // Don't exit, but return FALSE so default handler
        // gets called. The default handler, ExitProcess, will exit.
    default:
        return FALSE;
    }
}
#endif

int main(int argc, char** argv)
{
    return 0;
};
