#include "StdAfx.h"
#include "frontends/sdl/renderer/sdlrendererframe.h"
#include "frontends/sdl/utils.h"
#include "frontends/common2/programoptions.h"

#include "Interface.h"
#include "Core.h"

#include <iostream>

namespace sa2
{

  SDLRendererFrame::SDLRendererFrame(const common2::EmulatorOptions & options)
    : SDLFrame(options)
  {
    const common2::Geometry & geometry = options.geometry;

    myWindow.reset(SDL_CreateWindow(g_pAppTitle.c_str(), geometry.x, geometry.y, geometry.width, geometry.height, SDL_WINDOW_RESIZABLE), SDL_DestroyWindow);
    if (!myWindow)
    {
      throw std::runtime_error(SDL_GetError());
    }

    SetApplicationIcon();

    myRenderer.reset(SDL_CreateRenderer(myWindow.get(), options.sdlDriver, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC), SDL_DestroyRenderer);
    if (!myRenderer)
    {
      throw std::runtime_error(SDL_GetError());
    }

    const Uint32 format = SDL_PIXELFORMAT_ARGB8888;
    printRendererInfo(std::cerr, myRenderer, format, options.sdlDriver);

    Video & video = GetVideo();

    const int width = video.GetFrameBufferWidth();
    const int height = video.GetFrameBufferHeight();
    const int sw = video.GetFrameBufferBorderlessWidth();
    const int sh = video.GetFrameBufferBorderlessHeight();

    myTexture.reset(SDL_CreateTexture(myRenderer.get(), format, SDL_TEXTUREACCESS_STATIC, width, height), SDL_DestroyTexture);

    myRect.x = video.GetFrameBufferBorderWidth();
    myRect.y = video.GetFrameBufferBorderHeight();
    myRect.w = sw;
    myRect.h = sh;
    myPitch = width * sizeof(bgra_t);
  }

  void SDLRendererFrame::UpdateTexture()
  {
    SDL_UpdateTexture(myTexture.get(), nullptr, myFramebuffer.data(), myPitch);
  }

  void SDLRendererFrame::RenderPresent()
  {
    SDL_RenderCopyEx(myRenderer.get(), myTexture.get(), &myRect, nullptr, 0.0, nullptr, SDL_FLIP_VERTICAL);
    SDL_RenderPresent(myRenderer.get());
  }

}
