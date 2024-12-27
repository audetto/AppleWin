#include <StdAfx.h>

#include "frontends/libretro/rdirectsound.h"
#include "frontends/libretro/environment.h"

#include "linux/soundbuffer.h"

#include <unordered_map>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

namespace
{

  ra2::AudioSource getAudioSourceFromName(const std::string & name)
  {
    // These are the strings used in DSGetSoundBuffer

    if (name == "Spkr")
    {
      return ra2::AudioSource::SPEAKER;
    }

    if (name == "MB")
    {
      return ra2::AudioSource::MOCKINGBOARD;
    }

    if (name == "SSI263")
    {
      return ra2::AudioSource::SSI263;
    }

    // something new, just ignore it
    return ra2::AudioSource::UNKNOWN;
  }

  class SoundGenerator : public SoundBuffer
  {
  public:
    HRESULT Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName) override;
    HRESULT Release() override;

    void writeAudio(const size_t fps, const bool write);

    bool isRunning();

    ra2::AudioSource getSource() const;
    void setSource(ra2::AudioSource audioSource) { myAudioSource = audioSource; }

  private:
    ra2::AudioSource myAudioSource = ra2::AudioSource::UNKNOWN;
    std::vector<int16_t> myMixerBuffer;

    void mixBuffer(const void * ptr, const size_t size);
  };

  std::vector<SoundGenerator*> activeSoundGenerators;

  HRESULT SoundGenerator::Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName)
  {
    myAudioSource = getAudioSourceFromName(pDevName);
    return SoundBuffer::Init(dwFlags, dwBufferSize, nSampleRate, nChannels, pDevName);
  }

  HRESULT SoundGenerator::Release()
  {
    for (auto iter = activeSoundGenerators.begin(); iter != activeSoundGenerators.end(); ++iter)
    {
      if (*iter == this)
      {
        activeSoundGenerators.erase(iter);
        break;
      }
    }

    return SoundBuffer::Release();
  }

  bool SoundGenerator::isRunning()
  {
    DWORD dwStatus;
    GetStatus(&dwStatus);
    if (dwStatus & DSBSTATUS_PLAYING)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  ra2::AudioSource SoundGenerator::getSource() const
  {
    return myAudioSource;
  }

  void SoundGenerator::mixBuffer(const void * ptr, const size_t size)
  {
    const int16_t frames = size / (sizeof(int16_t) * myChannels);
    const int16_t * data = static_cast<const int16_t *>(ptr);

    if (myChannels == 2)
    {
      myMixerBuffer.assign(data, data + frames * myChannels);
    }
    else
    {
      myMixerBuffer.resize(2 * frames);
      for (int16_t i = 0; i < frames; ++i)
      {
        myMixerBuffer[i * 2] = data[i];
        myMixerBuffer[i * 2 + 1] = data[i];
      }
    }

    const double logVolume = GetLogarithmicVolume();
    // same formula as QAudio::convertVolume()
    const double linVolume = logVolume > 0.99 ? 1.0 : -std::log(1.0 - logVolume) / std::log(100.0);
    const int16_t rvolume = int16_t(linVolume * 128);

    for (int16_t & sample : myMixerBuffer)
    {
      sample = (sample * rvolume) / 128;
    }

    ra2::audio_batch_cb(myMixerBuffer.data(), frames);
  }

  void SoundGenerator::writeAudio(const size_t fps, const bool write)
  {
    const size_t frames = mySampleRate / fps;
    const size_t bytesToRead = frames * myChannels * sizeof(int16_t);

    LPVOID lpvAudioPtr1, lpvAudioPtr2;
    DWORD dwAudioBytes1, dwAudioBytes2;
    // always read to keep AppleWin audio algorithms working correctly.
    Read(bytesToRead, &lpvAudioPtr1, &dwAudioBytes1, &lpvAudioPtr2, &dwAudioBytes2);

    if (write)
    {
      if (lpvAudioPtr1 && dwAudioBytes1)
      {
        mixBuffer(lpvAudioPtr1, dwAudioBytes1);
      }
      if (lpvAudioPtr2 && dwAudioBytes2)
      {
        mixBuffer(lpvAudioPtr2, dwAudioBytes2);
      }
    }
  }

}

static SoundBufferBase* CreateSoundBuffer(void)
{
  SoundGenerator * generator = new SoundGenerator();
  activeSoundGenerators.push_back(generator);
  return generator;
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

namespace ra2
{

  void writeAudio(const AudioSource selectedSource, const size_t fps)
  {
    bool found = false;
    for (auto* generator : activeSoundGenerators)
    {
      if (generator->isRunning())
      {
        const bool selected = !found && (selectedSource == generator->getSource());
        // we still read audio from all buffers
        // to keep AppleWin audio generation woking correctly
        // but only write on the selected one
        generator->writeAudio(fps, selected);
        // TODO: implement an algorithm to merge 2 channels (speaker + mockingboard)
        found = found || selected;
      }
    }
    // TODO: if found = false, we should probably write some silence
  }

  void bufferStatusCallback(bool active, unsigned occupancy, bool underrun_likely)
  {
    if (active)
    {
      // I am not sure this is any useful
      static unsigned lastOccupancy = 0;
      const int diff = std::abs(int(lastOccupancy) - int(occupancy));
      if (diff >= 5)
      {
        // this is very verbose
        log_cb(RETRO_LOG_INFO, "RA2: %s occupancy = %d, underrun_likely = %d\n", __FUNCTION__, occupancy, underrun_likely);
        lastOccupancy = occupancy;
      }
    }
  }

}
