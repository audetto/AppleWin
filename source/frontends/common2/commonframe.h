#pragma once

#include "linux/linuxframe.h"

#include "Common.h"
#include "Configuration/Config.h"

#include "frontends/common2/speed.h"

namespace common2
{
    struct EmulatorOptions;

    class CommonFrame : public LinuxFrame
    {
    public:
        CommonFrame(const EmulatorOptions &options);

        void Begin() override;

        virtual void ResetSpeed();

        void ExecuteOneFrame(const int64_t microseconds);

        // this function will emulate GL vert sync if necessary
        // it acts as a syncronisation point (sa2 (in qemu) and applen)
        void SyncVideoPresentScreen(const int64_t microseconds);

        void ChangeMode(const AppMode_e mode);
        void TogglePaused();
        void SingleStep();

        void ResetHardware();
        bool HardwareChanged() const;

        void LoadSnapshot() override;

    protected:
        virtual void SetFullSpeed(const bool value);
        virtual bool CanDoFullSpeed();

        void ExecuteInRunningMode(const int64_t microseconds);
        void ExecuteInDebugMode(const int64_t microseconds);
        void Execute(const uint32_t uCycles);

        Speed mySpeed;

        // used to synchronise if OpenGL cannot do it (or without it)
        bool mySynchroniseWithTimer;
        std::chrono::time_point<std::chrono::steady_clock> myLastSync;

    private:
        const bool myAllowVideoUpdate;
        CConfigNeedingRestart myHardwareConfig;
    };

} // namespace common2
