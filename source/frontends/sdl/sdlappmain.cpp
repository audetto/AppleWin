#include "StdAfx.h"
#include "linux/benchmark.h"
#include "linux/context.h"
#include "linux/version.h"

#include "frontends/common2/fileregistry.h"
#include "frontends/common2/commoncontext.h"
#include "frontends/common2/argparser.h"
#include "frontends/common2/programoptions.h"
#include "frontends/common2/timer.h"
#include "frontends/sdl/gamepad.h"
#include "frontends/sdl/sdirectsound.h"
#include "frontends/sdl/sdlcompat.h"
#include "frontends/sdl/utils.h"

#include "frontends/sdl/renderer/sdlrendererframe.h"
#include "frontends/sdl/imgui/sdlimguiframe.h"

#include "Core.h"
#include "NTSC.h"
#include "Interface.h"

#include <iostream>
#include <iomanip>

// comment out to test / debug init / shutdown only
#define EMULATOR_RUN

namespace
{

    int getRefreshRate()
    {
        SDL_DisplayMode dummy;
        const SDL_DisplayMode *current = sa2::compat::getCurrentDisplayMode(dummy);
        return current->refresh_rate ? current->refresh_rate : 60;
    }

    struct AppData
    {
        const common2::EmulatorOptions myOptions;
        const LoggerContext myLogger;
        const RegistryContext mtRegistryContext;
        const std::shared_ptr<Paddle> myPaddle;

        std::shared_ptr<sa2::SDLFrame> myFrame;
        std::shared_ptr<const common2::CommonInitialisation> myInit;

        AppData(const common2::EmulatorOptions &options)
            : myOptions(options)
            , myLogger(options.log)
            , mtRegistryContext(CreateFileRegistry(options))
            , myPaddle(sa2::Gamepad::create(options.gameControllerIndex, options.gameControllerMappingFile))
        {
            if (options.imgui)
            {
                myFrame = std::make_shared<sa2::SDLImGuiFrame>(options);
            }
            else
            {
                myFrame = std::make_shared<sa2::SDLRendererFrame>(options);
            }

#ifdef __EMSCRIPTEN__
            SDL_SetHint(SDL_HINT_EMSCRIPTEN_KEYBOARD_ELEMENT, "#canvas");
#endif
            std::cerr << "GL swap interval: " << sa2::compat::getGLSwapInterval() << std::endl;

            myInit = std::make_shared<const common2::CommonInitialisation>(myFrame, myPaddle, options);
        }
    };

    class AppState
    {
    private:
        const std::shared_ptr<const AppData> myData;
        const int64_t myOneFrameMicros;

        common2::Timer myGlobal;
        common2::Timer myRefreshScreenTimer;
        common2::Timer myCpuTimer;
        common2::Timer myFrameTimer;

    public:
        AppState(const std::shared_ptr<const AppData> &data, const int fps)
            : myData(data)
            , myOneFrameMicros(1000000 / fps)
        {
        }

        SDL_AppResult loop()
        {
            myFrameTimer.tic();

            myCpuTimer.tic();
            myData->myFrame->ExecuteOneFrame(myOneFrameMicros);
            myCpuTimer.toc();

            if (!myData->myOptions.headless)
            {
                myRefreshScreenTimer.tic();
                if (g_bFullSpeed)
                {
                    myData->myFrame->VideoRedrawScreenDuringFullSpeed(g_dwCyclesThisFrame);
                }
                else
                {
                    myData->myFrame->SyncVideoPresentScreen(myOneFrameMicros);
                }
                myRefreshScreenTimer.toc();
            }

            myFrameTimer.toc();
            const bool quit = myData->myFrame->Quit();
            return quit ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
        }

        SDL_AppResult event(const SDL_Event &e)
        {
            bool quit = false;
            myData->myFrame->ProcessSingleEvent(e, quit);
            return quit ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
        }

        void end()
        {
            myGlobal.toc();

            std::cerr << "Global:  " << myGlobal << std::endl;
            std::cerr << "Frame:   " << myFrameTimer << std::endl;
            std::cerr << "Screen:  " << myRefreshScreenTimer << std::endl;
            std::cerr << "CPU:     " << myCpuTimer << std::endl;
        }
    };

    SDL_AppResult SDL_AppCreate(void **appstate, int argc, char **argv)
    {
        common2::EmulatorOptions options;
        const bool run = getEmulatorOptions(argc, argv, common2::OptionsType::sa2, "SDL2", options);
        if (!run)
        {
            return SDL_APP_SUCCESS;
        }

        std::cerr << std::fixed << std::setprecision(2);

        sa2::printVideoInfo(std::cerr);
        sa2::printAudioInfo(std::cerr);
        sa2::setAudioOptions(options);

        std::shared_ptr<const AppData> data = std::make_shared<const AppData>(options);

        const int fps = getRefreshRate();
        std::cerr << "Video refresh rate: " << fps << " Hz, " << 1000.0 / fps << " ms" << std::endl;

#ifdef EMULATOR_RUN
        if (options.benchmark)
        {
            // we need to switch off vsync, otherwise FPS is limited to 60
            // and it will take longer to run
            sa2::SDLFrame::setGLSwapInterval(0);

            const auto redraw = [&data] { data->myFrame->VideoPresentScreen(); };

            Video &video = GetVideo();
            const auto refresh = [redraw, &video]
            {
                NTSC_SetVideoMode(video.GetVideoMode());
                NTSC_VideoRedrawWholeScreen();
                redraw();
            };

            VideoBenchmark(redraw, refresh);
            return SDL_APP_SUCCESS;
        }
        else
        {
            *appstate = new AppState(data, fps);
            return SDL_APP_CONTINUE;
        }
#else
        return SDL_APP_SUCCESS;
#endif
    }

} // namespace

extern "C"
{
    SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv)
    {
        try
        {
#if SDL_VERSION_ATLEAST(3, 0, 0)
            // SDL_INIT_AUDIO is needed or it does not work
            // SDL_INIT_VIDEO allows the diagnostics to work
            if (!SA2_OK(SDL_InitSubSystem(SDL_INIT_AUDIO | SDL_INIT_VIDEO)))
            {
                throw std::runtime_error(sa2::decorateSDLError("SDL_InitSubSystem"));
            }

            const std::string version = getVersion();
            SDL_SetAppMetadata("AppleWin", version.c_str(), "org.applewin");
#endif

            return SDL_AppCreate(appstate, argc, argv);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return SDL_APP_FAILURE;
        }
    }

    SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
    {
        try
        {
            AppState *state = static_cast<AppState *>(appstate);
            return state->event(*event);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return SDL_APP_FAILURE;
        }
    }

    SDL_AppResult SDL_AppIterate(void *appstate)
    {
        try
        {
            AppState *state = static_cast<AppState *>(appstate);
            return state->loop();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return SDL_APP_FAILURE;
        }
    }

    void SDL_AppQuit(void *appstate, SDL_AppResult result)
    {
        AppState *state = static_cast<AppState *>(appstate);
        if (state)
        {
            state->end();
            delete state;
        }
    }
}
