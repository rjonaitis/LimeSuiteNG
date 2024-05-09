#include "USBTransferContext_FX3.h"

using namespace lime;

USBTransferContext_FX3::USBTransferContext_FX3()
    : USBTransferContext()
#ifndef __unix__
    , inOvLap(new OVERLAPPED)
    , EndPt(nullptr)
    , transferWaitedFor(false)
#endif
{
#ifndef __unix__
    memset(inOvLap, 0, sizeof(OVERLAPPED));
    inOvLap->hEvent = CreateEvent(NULL, false, false, NULL);
#endif
}

#ifndef __unix__
USBTransferContext_FX3::~USBTransferContext_FX3()
{
    CloseHandle(inOvLap->hEvent);
    delete inOvLap;
}
#endif

/**
  On Windows systems, resets the FX3 USB controller device.
  
  @return Opposite of `isTransferUsed`.
 */
bool USBTransferContext_FX3::Reset()
{
    if (isTransferUsed)
    {
        return false;
    }
#ifndef __unix__
    CloseHandle(inOvLap->hEvent);
    memset(inOvLap, 0, sizeof(OVERLAPPED));
    inOvLap->hEvent = CreateEvent(NULL, false, false, NULL);
#endif
    return true;
}
