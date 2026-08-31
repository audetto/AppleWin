#pragma once

#ifdef __EMSCRIPTEN__

namespace common2
{
    namespace emscripten
    {
        constexpr const char *STORAGE = "/storage";   // IDBFS persistent mount (SDL3)
        constexpr const char *DEFAULTS = "/defaults"; // read-only preloaded bundle
        constexpr const char *DISKS = "/disks";       // disk images & symbols
    } // namespace emscripten
} // namespace common2

#endif
