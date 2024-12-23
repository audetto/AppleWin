#pragma once

#ifdef _WIN32
#error libwindows must not be used on Windows builds.
#endif

#include "wincompat.h"
#include "guiddef.h"
#include "winhandles.h"
#include "wingdi.h"
#include "fileapi.h"
#include "timeapi.h"
#include "misc.h"
#include "strsafe.h"
#include "winstrings.h"
#include "gdi.h"
#include "winbase.h"
#include "winuser.h"
#include "dsound.h"
#include "winerror.h"
#include "mmreg.h"
#include "mmsystem.h"
#include "dmusicc.h"
#include "winnls.h"
#include "joystickapi.h"
