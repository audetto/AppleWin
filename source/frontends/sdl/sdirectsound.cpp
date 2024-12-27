#include "StdAfx.h"
#include "frontends/sdl/sdirectsound.h"
#include "frontends/sdl/utils.h"
#include "frontends/common2/programoptions.h"

#include "windows.h"

#include "Core.h"
#include "SoundCore.h"
#include "Log.h"

#include <SDL.h>

#include <unordered_map>
#include <memory>
#include <iostream>
#include <iomanip>

namespace
{

  // these have to come from EmulatorOptions
  std::string audioDeviceName;
  size_t audioBuffer = 0;

  size_t getBytesPerSecond(const SDL_AudioSpec & spec)
  {
    const size_t bitsPerSample = spec.format & SDL_AUDIO_MASK_BITSIZE;
    const size_t bytesPerFrame = spec.channels * bitsPerSample / 8;
    return spec.freq * bytesPerFrame;
  }

  size_t nextPowerOf2(size_t n)
  {
    size_t k = 1;
    while (k < n)
      k *= 2;
    return k;
  }

  class DirectSoundGenerator : public SoundBuffer
  {
  public:
    DirectSoundGenerator();
    virtual ~DirectSoundGenerator();
    virtual HRESULT Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName) override;
    virtual HRESULT Release() override;

    virtual HRESULT Stop() override;
    virtual HRESULT Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags ) override;

    void printInfo();
    sa2::SoundInfo getInfo();

  private:
    static void staticAudioCallback(void* userdata, uint8_t* stream, int len);

    void audioCallback(uint8_t* stream, int len);

    std::vector<uint8_t> myMixerBuffer;

    SDL_AudioDeviceID myAudioDevice;
    SDL_AudioSpec myAudioSpec;

    size_t myBytesPerSecond;
    std::string myName;

    uint8_t * mixBufferTo(uint8_t * stream);
  };

  std::unordered_map<DirectSoundGenerator *, std::shared_ptr<DirectSoundGenerator> > activeSoundGenerators;

  void DirectSoundGenerator::staticAudioCallback(void* userdata, uint8_t* stream, int len)
  {
    DirectSoundGenerator * generator = static_cast<DirectSoundGenerator *>(userdata);
    return generator->audioCallback(stream, len);
  }

  void DirectSoundGenerator::audioCallback(uint8_t* stream, int len)
  {
    LPVOID lpvAudioPtr1, lpvAudioPtr2;
    DWORD dwAudioBytes1, dwAudioBytes2;
    const size_t bytesRead = Read(len, &lpvAudioPtr1, &dwAudioBytes1, &lpvAudioPtr2, &dwAudioBytes2);

    myMixerBuffer.resize(bytesRead);

    uint8_t * dest = myMixerBuffer.data();
    if (lpvAudioPtr1 && dwAudioBytes1)
    {
      memcpy(dest, lpvAudioPtr1, dwAudioBytes1);
      dest += dwAudioBytes1;
    }
    if (lpvAudioPtr2 && dwAudioBytes2)
    {
      memcpy(dest, lpvAudioPtr2, dwAudioBytes2);
      dest += dwAudioBytes2;
    }

    stream = mixBufferTo(stream);

    const size_t gap = len - bytesRead;
    if (gap)
    {
      memset(stream, myAudioSpec.silence, gap);
    }
  }

  DirectSoundGenerator::DirectSoundGenerator()
    : SoundBuffer()
    , myAudioDevice(0)
    , myBytesPerSecond(0)
  {
  }

  HRESULT DirectSoundGenerator::Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName)
  {
    SDL_zero(myAudioSpec);

    SDL_AudioSpec want;
    SDL_zero(want);

    _ASSERT(audioBuffer > 0);

    want.freq = nSampleRate;
    want.format = AUDIO_S16LSB;
    want.channels = nChannels;
    want.samples = std::min<size_t>(MAX_SAMPLES, nextPowerOf2(nSampleRate * audioBuffer / 1000));
    want.callback = staticAudioCallback;
    want.userdata = this;

    const char * deviceName = audioDeviceName.empty() ? nullptr : audioDeviceName.c_str();
    myAudioDevice = SDL_OpenAudioDevice(deviceName, 0, &want, &myAudioSpec, 0);

    if (myAudioDevice)
    {
      myName = pDevName;
      myBytesPerSecond = getBytesPerSecond(myAudioSpec);
      return SoundBuffer::Init(dwFlags, dwBufferSize, nSampleRate, nChannels, pDevName);
    }

    LogOutput("DirectSoundGenerator: %s\n", sa2::decorateSDLError("SDL_OpenAudioDevice").c_str());
    return E_FAIL;
  }

  DirectSoundGenerator::~DirectSoundGenerator()
  {
    SDL_PauseAudioDevice(myAudioDevice, 1);
    SDL_CloseAudioDevice(myAudioDevice);
  }

  HRESULT DirectSoundGenerator::Release()
  {
    activeSoundGenerators.erase(this);  // this will force the destructor
    return DS_OK;
  }

  HRESULT DirectSoundGenerator::Stop()
  {
    const HRESULT res = SoundBuffer::Stop();
    SDL_PauseAudioDevice(myAudioDevice, 1);
    return res;
  }
  
  HRESULT DirectSoundGenerator::Play( DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags )
  {
    const HRESULT res = SoundBuffer::Play(dwReserved1, dwReserved2, dwFlags);
    SDL_PauseAudioDevice(myAudioDevice, 0);
    return res;
  }

  void DirectSoundGenerator::printInfo()
  {
    const uint32_t bytesInBuffer = GetBytesInBuffer();
    std::cerr << "Channels: " << (int)myAudioSpec.channels;
    std::cerr << ", buffer: " << std::setw(6) << bytesInBuffer;
    const double time = double(bytesInBuffer) / myBytesPerSecond * 1000;
    std::cerr << ", " << std::setw(8) << time << " ms";
    std::cerr << ", underruns: " << std::setw(10) << GetBufferUnderruns() << std::endl;
  }

  sa2::SoundInfo DirectSoundGenerator::getInfo()
  {
    DWORD dwStatus;
    GetStatus(&dwStatus);

    sa2::SoundInfo info;
    info.name = myName;
    info.running = dwStatus & DSBSTATUS_PLAYING;
    info.channels = myChannels;
    info.volume = GetLogarithmicVolume();
    info.numberOfUnderruns = GetBufferUnderruns();

    if (info.running && myBytesPerSecond > 0)
    {
      const uint32_t bytesInBuffer = GetBytesInBuffer();
      const float coeff = 1.0 / myBytesPerSecond;
      info.buffer = bytesInBuffer * coeff;
      info.size = myBufferSize * coeff;
    }

    return info;
  }

  uint8_t * DirectSoundGenerator::mixBufferTo(uint8_t * stream)
  {
    // we could copy ADJUST_VOLUME from SDL_mixer.c and avoid all copying and (rare) race conditions
    const double logVolume = GetLogarithmicVolume();
    // same formula as QAudio::convertVolume()
    const double linVolume = logVolume > 0.99 ? 1.0 : -std::log(1.0 - logVolume) / std::log(100.0);
    const uint8_t svolume = uint8_t(linVolume * SDL_MIX_MAXVOLUME);

    const size_t len = myMixerBuffer.size();
    memset(stream, 0, len);
    SDL_MixAudioFormat(stream, myMixerBuffer.data(), myAudioSpec.format, len, svolume);
    return stream + len;
  }

}

static SoundBufferBase* CreateSoundBuffer(void)
{
  try
  {
    DirectSoundGenerator * generator = new DirectSoundGenerator();
    activeSoundGenerators[generator].reset(generator);
    return generator;
  }
  catch (const std::exception & e)
  {
    g_bDisableDirectSound = true;
    g_bDisableDirectSoundMockingboard = true;
    LogOutput("SoundBuffer: %s\n", e.what());
    return nullptr;
  }
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

namespace sa2
{

  void printAudioInfo()
  {
    for (const auto & it : activeSoundGenerators)
    {
      const auto & generator = it.second;
      generator->printInfo();
    }
  }

  void resetAudioUnderruns()
  {
    for (const auto & it : activeSoundGenerators)
    {
      const auto & generator = it.second;
      generator->ResetUnderruns();
    }
  }

  std::vector<SoundInfo> getAudioInfo()
  {
    std::vector<SoundInfo> info;
    info.reserve(activeSoundGenerators.size());

    for (const auto & it : activeSoundGenerators)
    {
      const auto & generator = it.second;
      info.push_back(generator->getInfo());
    }

    return info;
  }

  void setAudioOptions(const common2::EmulatorOptions & options)
  {
    audioDeviceName = options.audioDeviceName;
    audioBuffer = options.audioBuffer;
  }

}
