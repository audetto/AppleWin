#include "StdAfx.h"

#include "Configuration/PageSlots.h"

// IPropertySheetPage
INT_PTR CPageSlots::DlgProcInternal(HWND hWnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    return 0;
}

void CPageSlots::DlgOK(HWND hWnd)
{
}

void CPageSlots::ApplyConfigAfterClose()
{
}

void CPageSlots::DlgCANCEL(HWND hWnd)
{
}

void CPageSlots::ResetToDefault()
{
}

UINT CPageSlots::ms_slot = 0;
CPageSlots *CPageSlots::ms_this = nullptr;
