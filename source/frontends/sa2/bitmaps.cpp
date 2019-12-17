#include <SDL.h>
#include <memory>
#include <iostream>

#include <linux/interface.h>
#include <linux/windows/misc.h>

struct CBITMAP : public CHANDLE
{
  std::shared_ptr<SDL_Surface> surface;
};

std::string getPath(const std::string & resource)
{
  if (resource == "CHARSET40") return "resource/CHARSET4.BMP";
  if (resource == "CHARSET82") return "resource/CHARSET82.bmp";
  if (resource == "CHARSET8M") return "resource/CHARSET8M.bmp";
  if (resource == "CHARSET8C") return "resource/CHARSET8C.bmp";

  return resource;
}

HBITMAP LoadBitmap(HINSTANCE hInstance, const std::string & resource)
{
  const std::string path = getPath(resource);
  std::shared_ptr<SDL_Surface> surface(SDL_LoadBMP(path.c_str()), SDL_FreeSurface);
  if (surface)
  {
    CBITMAP * bitmap = new CBITMAP;
    bitmap->surface = surface;
    return bitmap;
  }
  else
  {
    std::cerr << "Cannot load: " << resource << " with path: " << path << std::endl;
    return nullptr;
  }
}

LONG GetBitmapBits(HBITMAP hbit, LONG cb, LPVOID lpvBits)
{
  const CBITMAP & bitmap = dynamic_cast<CBITMAP&>(*hbit);

  SDL_LockSurface(bitmap.surface.get());

  const SDL_Surface * surface = bitmap.surface.get();

  const char * source = static_cast<char *>(surface->pixels);
  const size_t size = surface->h * surface->w / 8;
  const size_t requested = cb;

  const size_t copied = std::min(requested, size);

  char * dest = static_cast<char *>(lpvBits);

  for (size_t i = 0; i < copied; ++i)
  {
    const size_t offset = i * 8;
    char val = 0;
    for (size_t j = 0; j < 8; ++j)
    {
      const char pixel = *(source + offset + j);
      val = (val << 1) | pixel;
    }
    dest[i] = val;
  }

  SDL_UnlockSurface(bitmap.surface.get());

  return copied;
}