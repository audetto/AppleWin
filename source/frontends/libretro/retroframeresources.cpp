#include "StdAfx.h"
#include "frontends/libretro/retroframe.h"
#include "frontends/libretro/environment.h"

#include "linux/resources.h"

#include "../resource/resource.h"
#include "rom_Apple2.inl"
#include "rom_Apple2_Plus.inl"
#include "rom_Apple2_Video.inl"
#include "rom_Apple2_JPlus.inl"
#include "rom_Apple2_JPlus_Video.inl"
#include "rom_Apple2e.inl"
#include "rom_Apple2e_Enhanced.inl"
#include "rom_Apple2e_Enhanced_Video.inl"
#include "rom_Base64A.inl"
#include "rom_Base64A_German_Video.inl"
#include "rom_DISK2.inl"
#include "rom_DISK2_13sector.inl"
#include "rom_Mockingboard_D.inl"
#include "rom_MouseInterface.inl"
#include "rom_Parallel.inl"
#include "rom_PRAVETS82.inl"
#include "rom_PRAVETS8C.inl"
#include "rom_PRAVETS8M.inl"
#include "rom_SSC.inl"
#include "rom_ThunderClockPlus.inl"
#include "rom_TK3000e.inl"
#include "rom_TKClock.inl"

#include "bin_Hddrvr.inl"
#include "bin_Hddrvr_v2.inl"
#include "bin_HDC_SmartPort.inl"

#include "bmp_CHARSET82.inl"
#include "bmp_CHARSET8C.inl"
#include "bmp_CHARSET8M.inl"

namespace ra2
{

  void RetroFrame::GetBitmap(LPCSTR lpBitmapName, LONG cb, LPVOID lpvBits)
  {
    if (strcmp(lpBitmapName, "CHARSET8C") == 0)
      memcpy(lpvBits, _bmp_CHARSET8C, cb);
    else if (strcmp(lpBitmapName, "CHARSET8M") == 0)
      memcpy(lpvBits, _bmp_CHARSET8M, cb);
    else if (strcmp(lpBitmapName, "CHARSET82") == 0)
      memcpy(lpvBits, _bmp_CHARSET82, cb);
    else
    {
      log_cb(RETRO_LOG_INFO, "RA2: %s. Missing bitmap '%s'\n", __FUNCTION__, lpBitmapName);
      memset(lpvBits, 0, cb);
    }
  }

  BYTE* RetroFrame::GetResource(WORD id, LPCSTR lpType, DWORD expectedSize)
  {
    switch (id)
    {
    case IDR_APPLE2_ROM:
      return _rom_Apple2;
    case IDR_APPLE2_PLUS_ROM:
      return _rom_Apple2_Plus;
    case IDR_APPLE2_VIDEO_ROM:
      return _rom_Apple2_Video;
    case IDR_APPLE2_JPLUS_ROM:
      return _rom_Apple2_JPlus;
    case IDR_APPLE2_JPLUS_VIDEO_ROM:
      return _rom_Apple2_JPlus_Video;
    case IDR_APPLE2E_ROM:
      return _rom_Apple2e;
    case IDR_APPLE2E_ENHANCED_ROM:
      return _rom_Apple2e_Enhanced;
    case IDR_APPLE2E_ENHANCED_VIDEO_ROM:
      return _rom_Apple2e_Enhanced_Video;
    case IDR_BASE_64A_ROM:
      return _rom_Base64A;
    case IDR_BASE64A_VIDEO_ROM:
      return _rom_Base64A_German_Video;
    case IDR_DISK2_16SECTOR_FW:
      return _rom_DISK2;
    case IDR_DISK2_13SECTOR_FW:
      return _rom_DISK2_13sector;
    case IDR_HDC_SMARTPORT_FW:
      return _bin_HDC_SmartPort;
    case IDR_HDDRVR_FW:
      return _bin_Hddrvr;
    case IDR_HDDRVR_V2_FW:
      return _bin_Hddrvr_v2;
    case IDR_MOCKINGBOARD_D_FW:
      return _rom_Mockingboard_D;
    case IDR_MOUSEINTERFACE_FW:
      return _rom_MouseInterface;
    case IDR_PRAVETS_82_ROM:
      return _rom_PRAVETS82;
    case IDR_PRAVETS_8M_ROM:
      return _rom_PRAVETS8M;
    case IDR_PRAVETS_8C_ROM:
      return _rom_PRAVETS8C;
    case IDR_PRINTDRVR_FW:
      return _rom_Parallel;
    case IDR_SSC_FW:
      return _rom_SSC;
    case IDR_TK3000_2E_ROM:
      return _rom_TK3000e;
    case IDR_TKCLOCK_FW:
      return _rom_TKClock;
    case IDR_THUNDERCLOCKPLUS_FW:
      return _rom_ThunderClockPlus;
    default:
      log_cb(RETRO_LOG_INFO, "RA2: %s. Missing resource %u: '%s'\n", __FUNCTION__, id, getResourceName(id).c_str());
      return NULL;
    }
  }
}
