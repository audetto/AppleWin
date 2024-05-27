#pragma once

#include "sa2_config.h"

// SDL2 / SDL3 compatibility layer
// some #ifdefs are still present in the code, where this is easier to handle

#ifdef SA2_SDL3
  #include <SDL3/SDL.h>
  #include <SDL3/SDL_opengl.h>

  #define ImGui_ImplSDLX_InitForOpenGL ImGui_ImplSDL3_InitForOpenGL
  #define ImGui_ImplSDLX_Shutdown ImGui_ImplSDL3_Shutdown
  #define ImGui_ImplSDLX_NewFrame ImGui_ImplSDL3_NewFrame
  #define ImGui_ImplSDLX_ProcessEvent ImGui_ImplSDL3_ProcessEvent

  #define SA2_CONTROLLER_BUTTON(e) e.gbutton.button
  #define SA2_DROP_FILE(d) d.data
  #define SA2_RENDERER_MODE , SDL_LOGICAL_PRESENTATION_STRETCH, SDL_SCALEMODE_NEAREST
  #define SA2_IMAGE_BITS(s) s->format->bits_per_pixel

  typedef SDL_FRect Renderer_Rect_t;
#else
  #include <SDL.h>
  #include <SDL_opengl.h>

  #define ImGui_ImplSDLX_InitForOpenGL ImGui_ImplSDL2_InitForOpenGL
  #define ImGui_ImplSDLX_Shutdown ImGui_ImplSDL2_Shutdown
  #define ImGui_ImplSDLX_NewFrame ImGui_ImplSDL2_NewFrame
  #define ImGui_ImplSDLX_ProcessEvent ImGui_ImplSDL2_ProcessEvent

  #define SA2_CONTROLLER_BUTTON(e) e.cbutton.button
  #define SA2_DROP_FILE(d) d.file
  #define SA2_RENDERER_MODE

  #define SA2_IMAGE_BITS(s) s->format->BitsPerPixel

  typedef SDL_Rect Renderer_Rect_t;
#endif

namespace common2
{
  struct Geometry;
}

namespace sa2
{
  namespace compat
  {

    int getNumJoysticks();
    int getGLSwapInterval();
    const SDL_DisplayMode * getCurrentDisplayMode();
    void pauseAudioDevice(SDL_AudioDeviceID dev);
    void resumeAudioDevice(SDL_AudioDeviceID dev);
    void toggleCursor();
    SDL_Window * createWindow(const char *title, const common2::Geometry & geometry, SDL_WindowFlags flags);
    SDL_Renderer * createRenderer(SDL_Window * window, const int index);

  }
}
