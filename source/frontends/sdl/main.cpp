#include <iostream>
#include <memory>
#include <iomanip>

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

#ifdef __EMSCRIPTEN__
#include "examples/libs/emscripten/emscripten_mainloop_stub.h"
#endif

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

    struct Data
    {
        sa2::SDLFrame *frame;
        SDL_mutex *mutex;
        common2::Timer *timer;
    };

    class MainLoop
    {
    public:
        MainLoop(std::shared_ptr<sa2::SDLFrame> &aframe, common2::EmulatorOptions aoptions, const int fps)
            : frame(aframe)
            , options(aoptions)
            , oneFrameMicros(1000000 / fps)
        {
        }

        ~MainLoop()
        {
            // printf("%s:%d\n", __FILE__, __LINE__);
        }

        void loop()
        {
            frameTimer.tic();

            eventTimer.tic();
            frame->ProcessEvents(quit);
            eventTimer.toc();

            cpuTimer.tic();
            frame->ExecuteOneFrame(oneFrameMicros);
            cpuTimer.toc();

            if (!options.headless)
            {
                refreshScreenTimer.tic();
                if (g_bFullSpeed)
                {
                    frame->VideoRedrawScreenDuringFullSpeed(g_dwCyclesThisFrame);
                }
                else
                {
                    frame->SyncVideoPresentScreen(oneFrameMicros);
                }
                refreshScreenTimer.toc();
            }

            frameTimer.toc();
        }

        void end()
        {
            global.toc();

            std::cerr << "Global:  " << global << std::endl;
            std::cerr << "Frame:   " << frameTimer << std::endl;
            std::cerr << "Screen:  " << refreshScreenTimer << std::endl;
            std::cerr << "Events:  " << eventTimer << std::endl;
            std::cerr << "CPU:     " << cpuTimer << std::endl;
        }

        bool isQuit() const
        {
            return quit || frame->Quit();
        }

    private:
        const std::shared_ptr<sa2::SDLFrame> frame;
        const common2::EmulatorOptions options;
        const int64_t oneFrameMicros;
        bool quit = false;

        common2::Timer global;
        common2::Timer refreshScreenTimer;
        common2::Timer cpuTimer;
        common2::Timer eventTimer;
        common2::Timer frameTimer;
    };

    std::unique_ptr<MainLoop> mainLoopPtr;

} // namespace

void run_sdl(int argc, char *const argv[])
{
    common2::EmulatorOptions options;

    const bool run = getEmulatorOptions(argc, argv, common2::OptionsType::sa2, "SDL2", options);

    if (!run)
        return;

    std::cerr << std::fixed << std::setprecision(2);

    sa2::printVideoInfo(std::cerr);
    sa2::printAudioInfo(std::cerr);

    const LoggerContext logger(options.log);
    const RegistryContext registryContext(CreateFileRegistry(options));
    const std::shared_ptr<Paddle> paddle =
        sa2::Gamepad::create(options.gameControllerIndex, options.gameControllerMappingFile);

    sa2::setAudioOptions(options);

    std::shared_ptr<sa2::SDLFrame> frame;
    if (options.imgui)
    {
        frame = std::make_shared<sa2::SDLImGuiFrame>(options);
    }
    else
    {
        frame = std::make_shared<sa2::SDLRendererFrame>(options);
    }

#ifdef __EMSCRIPTEN__
    SDL_SetHint(SDL_HINT_EMSCRIPTEN_KEYBOARD_ELEMENT, "#canvas");
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
#endif

    std::cerr << "GL swap interval: " << sa2::compat::getGLSwapInterval() << std::endl;

    const common2::CommonInitialisation init(frame, paddle, options);

    const int fps = getRefreshRate();
    std::cerr << "Video refresh rate: " << fps << " Hz, " << 1000.0 / fps << " ms" << std::endl;

#ifdef EMULATOR_RUN
    if (options.benchmark)
    {
        // we need to switch off vsync, otherwise FPS is limited to 60
        // and it will take longer to run
        sa2::SDLFrame::setGLSwapInterval(0);

        const auto redraw = [&frame] { frame->VideoPresentScreen(); };

        Video &video = GetVideo();
        const auto refresh = [redraw, &video]
        {
            NTSC_SetVideoMode(video.GetVideoMode());
            NTSC_VideoRedrawWholeScreen();
            redraw();
        };

        VideoBenchmark(redraw, refresh);
    }
    else
    {
        mainLoopPtr = std::make_unique<MainLoop>(frame, options, fps);

#ifdef __EMSCRIPTEN__
        EMSCRIPTEN_MAINLOOP_BEGIN
#else
        while (!mainLoopPtr->isQuit())
#endif
        {

#ifdef __EMSCRIPTEN__
            if (mainLoopPtr->isQuit())
            {
                emscripten_cancel_main_loop();
            }
#endif
            try
            {
                mainLoopPtr->loop();
            }
            catch (const std::exception &e)
            {
                std::cerr << "Exception caught in main loop: " << e.what() << std::endl;
            }
        }
#ifdef __EMSCRIPTEN__
        EMSCRIPTEN_MAINLOOP_END;
#endif
        mainLoopPtr->end();
        mainLoopPtr.reset();
    }
#endif
}

int app_main(int argc, char *argv[])
{
#if SDL_VERSION_ATLEAST(3, 0, 0)
    const std::string version = getVersion();
    SDL_SetAppMetadata("AppleWin", version.c_str(), "org.applewin");
#endif

    // First we need to start up SDL, and make sure it went ok
    const Uint32 flags = SDL_INIT_VIDEO | SA2_INIT_GAMEPAD | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_JOYSTICK;

    if (!SA2_OK(SDL_Init(flags)))
    {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    int exit = 0;

    try
    {
        run_sdl(argc, argv);
    }
    catch (const std::exception &e)
    {
        exit = 2;
        std::cerr << e.what() << std::endl;
    }

    SDL_Quit();

    return exit;
}

#ifdef __EMSCRIPTEN__
extern "C" EMSCRIPTEN_KEEPALIVE void on_fs_ready()
{
    app_main(0, nullptr);
}
#else
int main(int argc, char *argv[])
{
    return app_main(argc, argv);
}
#endif
