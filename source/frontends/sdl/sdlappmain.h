#pragma once

#include "frontends/sdl/sdlcompat.h"

extern "C"
{
    SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv);
    SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event);
    SDL_AppResult SDL_AppIterate(void *appstate);
    void SDL_AppQuit(void *appstate, SDL_AppResult result);
}
