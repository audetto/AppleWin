#include "StdAfx.h"

#include "Configuration/PageInput.h"

// IPropertySheetPage
INT_PTR CPageInput::DlgProcInternal(HWND hWnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    return 0;
}

void CPageInput::DlgOK(HWND hWnd)
{
}

void CPageInput::ApplyConfigAfterClose()
{
}

void CPageInput::ResetToDefault()
{
}

CPageInput *CPageInput::ms_this = nullptr;
