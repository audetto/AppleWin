#include "StdAfx.h"
#include "frontends/common2/gnuframe.h"
#include "frontends/common2/utils.h"
#include "apple2roms_data.h"

#ifdef __APPLE__
#include "mach-o/dyld.h"
#endif

#include "Core.h"
#include "common_config.h"

#include <filesystem>

namespace
{

    std::filesystem::path getExecutableFilename()
    {
        std::filesystem::path executable;
#ifdef _WIN32
        char self[1024] = {0};
        int ch = GetModuleFileNameA(0, self, sizeof(self));
        if (ch < sizeof(self))
        {
            executable = self;
        }
#elif __APPLE__
        char self[1024] = {0};
        uint32_t size = sizeof(self);
        const int ch = _NSGetExecutablePath(self, &size);
        if (ch != -1)
        {
            executable = self;
        }
#else
        executable = std::filesystem::read_symlink("/proc/self/exe");
#endif
        return executable;
    }

    std::filesystem::path getResourceFolder(const std::string &target)
    {
        const std::filesystem::path executable = getExecutableFilename();

        for (const auto &rel : {SHARE_PATH, ROOT_PATH})
        {
            const std::filesystem::path path = executable.parent_path() / rel / target;
            {
                if (std::filesystem::exists(path))
                {
                    const auto resourcePath = std::filesystem::canonical(path);
                    return resourcePath;
                }
            }
        }

        // this is now used only for g_sProgramDir
        // which only matters for Debug Symbols and Printer Filename
        // let's be tolerant and use cwd

        return std::filesystem::current_path() / target;
    }

} // namespace

namespace common2
{

    GNUFrame::GNUFrame(const EmulatorOptions &options)
        : CommonFrame(options)
    {
        // should this go down to LinuxFrame (maybe Initialisation?)
        g_sProgramDir = getResourceFolder("bin").string() + PATH_SEPARATOR;
        LogFileOutput("Program Dir: '%s'\n", g_sProgramDir.c_str());
    }

    std::pair<const unsigned char *, unsigned int> GNUFrame::GetResourceData(WORD id) const
    {
        const auto it = apple2roms::data.find(id);
        if (it == apple2roms::data.end())
        {
            throw std::runtime_error("Cannot locate resource: " + std::to_string(id));
        }
        return it->second;
    }

    std::string GNUFrame::Video_GetScreenShotFolder() const
    {
        const std::filesystem::path pictures = getHomeDir() / "Pictures";
        std::filesystem::create_directories(pictures);
        return pictures.string() + PATH_SEPARATOR;
    }

} // namespace common2
