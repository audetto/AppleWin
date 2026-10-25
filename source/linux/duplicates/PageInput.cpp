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

void CPageInput::SetShiftKeyModDefault(const eApple2Type type)
{
}

bool CPageInput::GetShiftKeyModDefault(const eApple2Type type)
{
    return false;
}

CPageInput *CPageInput::ms_this = nullptr;
