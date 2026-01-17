#ifdef __EMSCRIPTEN__

#include "frontends/sdl/sdlcompat.h"
#include <emscripten.h>

extern "C" EMSCRIPTEN_KEEPALIVE void sdl_dropfile(const char *filename)
{
    SDL_Event e;
    SDL_zero(e);

    e.type = SDL_DROPFILE;
    // TODO
    e.drop.data = SDL_strdup(filename); // SDL requires this

    printf("[DND] pushing SDL_DROPFILE event for '%s'\n", filename);

    SDL_PushEvent(&e);
}

#endif
