#include "frontends/sdl/sdlappmain.h"

#if SDL_VERSION_ATLEAST(3, 0, 0)

#define SDL_MAIN_USE_CALLBACKS
#include <SDL3/SDL_main.h>

#else

int main(int argc, char **argv)
{
    void *appstate = nullptr;
    SDL_AppResult final_result = SDL_APP_FAILURE;

    const Uint32 flags = SDL_INIT_VIDEO | SA2_INIT_GAMEPAD | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_JOYSTICK;
    if (SDL_Init(flags) != 0)
    {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_AppResult rc = SDL_AppInit(&appstate, argc, argv);
    if (rc != SDL_APP_CONTINUE)
    {
        final_result = rc;
        SDL_AppQuit(appstate, final_result);
        SDL_Quit();
        return (rc == SDL_APP_SUCCESS) ? 0 : 1;
    }

    bool running = true;
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            rc = SDL_AppEvent(appstate, &event);
            if (rc != SDL_APP_CONTINUE)
            {
                final_result = rc;
                running = false;
                break;
            }
        }

        if (!running)
        {
            break;
        }

        rc = SDL_AppIterate(appstate);
        if (rc != SDL_APP_CONTINUE)
        {
            final_result = rc;
            break;
        }
    }

    SDL_AppQuit(appstate, final_result);
    SDL_Quit();
    return (final_result == SDL_APP_SUCCESS) ? 0 : 1;
}

#endif
