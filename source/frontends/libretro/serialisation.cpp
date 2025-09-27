#include "StdAfx.h"
#include "SaveState.h"

#include "frontends/libretro/serialisation.h"
#include "frontends/libretro/diskcontrol.h"

#include <cstdio>
#include <fstream>
#include <unistd.h>

#ifdef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#endif

namespace
{
    class AutoFile
    {
    public:
        AutoFile();
        ~AutoFile();

        const std::string &getFilename() const; // only if true

    protected:
        int myFD;
        std::string myFilename;
    };

    AutoFile::AutoFile()
    {
#ifdef _WIN32
        char tempPath[MAX_PATH], pattern[MAX_PATH];
        UINT result = GetTempPathA(MAX_PATH, tempPath);
        if (result == 0 || result > MAX_PATH)
        {
            throw std::runtime_error("Could not determine temp path");
        }
        GetTempFileNameA(tempPath, "aw", 0, pattern);
        if (result == 0 || result > MAX_PATH)
        {
            throw std::runtime_error("Could not generate temporary filename");
        }
        myFD = open(pattern, O_RDWR | O_CREAT, _S_IREAD | _S_IWRITE);
#else
        char pattern[] = "/tmp/awXXXXXX.aws.yaml";
        myFD = mkstemps(pattern, 9);
#endif
        if (myFD <= 0)
        {
            throw std::runtime_error("Cannot create temporary file");
        }
        myFilename = pattern;
    }

    AutoFile::~AutoFile()
    {
        close(myFD);
        std::remove(myFilename.c_str());
    }

    const std::string &AutoFile::getFilename() const
    {
        return myFilename;
    }

    void saveToFile(const std::string &filename) // cannot be null!
    {
        Snapshot_SetFilename(filename);
        Snapshot_SaveState();
    }

} // namespace

namespace ra2
{

    size_t RetroSerialisation::getSize()
    {
        AutoFile autoFile;
        std::string const &filename = autoFile.getFilename();
        saveToFile(filename);
        std::ifstream ifs(filename, std::ios::binary | std::ios::ate);

        const size_t fileSize = ifs.tellg();
        // we add a buffer to include a few things
        // DiscControl images
        // various sizes
        // small variations in AW yaml format
        const size_t buffer = 4096;
        return fileSize + buffer;
    }

    void RetroSerialisation::serialise(void *data, size_t size, const DiskControl &diskControl)
    {
        Buffer buffer(reinterpret_cast<char *>(data), size);
        diskControl.serialise(buffer);

        AutoFile autoFile;
        std::string const &filename = autoFile.getFilename();
        saveToFile(filename);
        std::ifstream ifs(filename, std::ios::binary | std::ios::ate);

        size_t const fileSize = ifs.tellg();
        buffer.get<size_t>() = fileSize;

        char *begin, *end;
        buffer.get(fileSize, begin, end);

        ifs.seekg(0, std::ios::beg);
        ifs.read(begin, end - begin);
    }

    void RetroSerialisation::deserialise(const void *data, size_t size, DiskControl &diskControl)
    {
        Buffer buffer(reinterpret_cast<const char *>(data), size);
        diskControl.deserialise(buffer);

        const size_t fileSize = buffer.get<size_t const>();

        AutoFile autoFile;
        std::string const &filename = autoFile.getFilename();
        // do not remove the {} scope below! it ensures the file is flushed
        {
            char const *begin, *end;
            buffer.get(fileSize, begin, end);
            std::ofstream ofs(filename, std::ios::binary);
            ofs.write(begin, end - begin);
        }

        // bit of a workaround, since the state files do not have full disk paths
        SetCurrentDirectory(diskControl.getCurrentDiskFolder().c_str());
        Snapshot_SetFilename(filename);
        Snapshot_LoadState();
    }

} // namespace ra2
