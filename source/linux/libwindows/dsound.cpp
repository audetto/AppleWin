#include "dsound.h"

#include "linux/soundbuffer.h"

#include <cstring>

static SoundBufferBase* CreateSoundBuffer(void)
{
  return new SoundBuffer();
}

extern bool g_bDSAvailable;

bool DSInit()
{
  SoundBufferBase::Create = CreateSoundBuffer;
  g_bDSAvailable = true;
  return true;
}

void DSUninit()
{
}
