#pragma once

#include "frontends/common2/gnuframe.h"

#include <vector>

namespace ra2
{

    class RetroFrame : public common2::GNUFrame
    {
    public:
        RetroFrame(const common2::EmulatorOptions &options, const size_t linePeriod);

        void VideoPresentScreen() override;
        void FrameRefreshStatus(int drawflags) override;
        void Initialize(bool resetVideoState) override;
        void Destroy() override;
        void Begin() override;
        void End() override;
        int FrameMessageBox(LPCSTR lpText, LPCSTR lpCaption, UINT uType) override;

        std::shared_ptr<SoundBuffer> CreateSoundBuffer(
            uint32_t dwBufferSize, uint32_t nSampleRate, int nChannels, const char *pszVoiceName) override;

        size_t GetFrameBufferLinePeriod() const;
        LPBYTE GetMainMemoryReference() const;

    protected:
        virtual void SetFullSpeed(const bool value) override;
        virtual bool CanDoFullSpeed() override;

    private:
        const size_t myLinePeriod;

        std::vector<uint8_t> myVideoBuffer;

        size_t myPitch = 0;
        size_t myOffset = 0;
        size_t myHeight = 0;
        size_t myBorderlessWidth = 0;
        size_t myBorderlessHeight = 0;
        uint8_t *myFrameBuffer = nullptr;

        LPBYTE myMainMemoryReference = nullptr;
    };

} // namespace ra2
