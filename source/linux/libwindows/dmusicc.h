#pragma once

#include "winhandles.h"
#include "guiddef.h"

typedef LONGLONG REFERENCE_TIME;
struct IReferenceClock : public IAutoRelease
{
    HRESULT GetTime(REFERENCE_TIME *pTime);
    HRESULT AdvisePeriodic(
        REFERENCE_TIME startTime, REFERENCE_TIME periodTime, HSEMAPHORE hSemaphore, DWORD_PTR *pdwAdviseCookie);
    HRESULT Unadvise(DWORD_PTR dwAdviseCookie);
};
