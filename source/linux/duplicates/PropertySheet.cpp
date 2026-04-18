#include "StdAfx.h"

#include "Configuration/PropertySheet.h"

void CPropertySheet::Init(void)
{
}

uint32_t CPropertySheet::GetVolumeMax(void)
{
    return 99;
}

bool CPropertySheet::SaveStateSelectImage(HWND hWindow, bool bSave)
{
    return false;
}

void CPropertySheet::ResetAllToDefault()
{
}

void CPropertySheet::ApplyConfigAfterClose(UINT bmPages)
{
}
