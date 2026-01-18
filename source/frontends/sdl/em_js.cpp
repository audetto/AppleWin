#ifdef __EMSCRIPTEN__

#include "frontends/sdl/sdlcompat.h"
#include <emscripten.h>

namespace
{
    void push_simple_event(SDL_EventType type)
    {
        SDL_Event e;
        SDL_zero(e);
        e.type = type;
        SDL_PushEvent(&e);
    }

} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE void sdl_dropfile(const char *filename)
{
    push_simple_event(SDL_DROPBEGIN);

    SDL_Event e;
    SDL_zero(e);
    e.type = SDL_DROPFILE;
    e.drop.data = SDL_strdup(filename); // SDL3 will free this memory
    printf("[DND] pushing SDL_DROPFILE event for '%s'\n", filename);
    SDL_PushEvent(&e);

    push_simple_event(SDL_DROPCOMPLETE);
}

#endif
