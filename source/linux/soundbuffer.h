#pragma once

#include "SoundBufferBase.h"

#include <vector>
#include <mutex>
#include <atomic>

class SoundBuffer : public SoundBufferBase
{
private:
  std::vector<uint8_t> mySoundBuffer;

  size_t myPlayPosition = 0;
  size_t myWritePosition = 0;
  WORD myStatus = 0;
  LONG myVolume = 0;

  // updated by the callback
  std::atomic_size_t myNumberOfUnderruns;
  std::mutex myMutex;

public:
  size_t myBufferSize;
  size_t mySampleRate;
  size_t myChannels;
  size_t myBitsPerSample;

  HRESULT Init(DWORD dwFlags, DWORD dwBufferSize, DWORD nSampleRate, int nChannels, LPCSTR pDevName) override;
  HRESULT Release() override;

  HRESULT SetCurrentPosition(DWORD dwNewPosition) override;
  HRESULT GetCurrentPosition(LPDWORD lpdwCurrentPlayCursor, LPDWORD lpdwCurrentWriteCursor) override;

  HRESULT Lock(DWORD dwWriteCursor, DWORD dwWriteBytes, LPVOID* lplpvAudioPtr1, DWORD* lpdwAudioBytes1, LPVOID* lplpvAudioPtr2, DWORD* lpdwAudioBytes2, DWORD dwFlags) override;
  virtual HRESULT Unlock(LPVOID lpvAudioPtr1, DWORD dwAudioBytes1, LPVOID lpvAudioPtr2, DWORD dwAudioBytes2) override;

  virtual HRESULT Stop() override;
  virtual HRESULT Play(DWORD dwReserved1, DWORD dwReserved2, DWORD dwFlags) override;

  virtual HRESULT SetVolume(LONG lVolume) override;
  HRESULT GetVolume(LONG* lplVolume) override;

  HRESULT GetStatus(LPDWORD lpdwStatus) override;
  HRESULT Restore() override;

  DWORD Read(DWORD dwReadBytes, LPVOID* lplpvAudioPtr1, DWORD* lpdwAudioBytes1, LPVOID* lplpvAudioPtr2, DWORD* lpdwAudioBytes2);
  DWORD GetBytesInBuffer();
  size_t GetBufferUnderruns() const;
  void ResetUnderruns();
  double GetLogarithmicVolume() const;  // in [0, 1]
};