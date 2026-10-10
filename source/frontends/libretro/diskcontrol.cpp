#include "StdAfx.h"
#include "frontends/libretro/diskcontrol.h"
#include "frontends/libretro/environment.h"
#include "frontends/libretro/retroregistry.h"

#include "Core.h"
#include "CardManager.h"
#include "Disk.h"
#include "Harddisk.h"

#include <fstream>
#include <filesystem>

namespace
{

    const std::string M3U_COMMENT("#");
    const std::string M3U_LABEL("#LABEL:");
    const std::string M3U_EXTINF("#EXTINF:");
    const std::string M3U_SAVEDISK("#SAVEDISK:");

    const std::string SAVEDISK_LABEL("Save Disk ");

    // Game disks are write-protected unless the option lets a game write to them
    bool gameDiskWriteProtection()
    {
        return ra2::getFloppyWriteProtect() ? IMAGE_FORCE_WRITE_PROTECTED : IMAGE_USE_FILES_WRITE_PROTECT_STATUS;
    }

    // A game disk that may be written is used from a copy in the save folder, so
    // the original keeps the checksum RetroAchievements identifies the game by
    bool copyToSaveFolder(const std::string &path, std::string &copyPath, bool &created)
    {
        created = false;
        if (ra2::save_directory.empty())
        {
            return false;
        }

        const std::filesystem::path original(path);
        const std::filesystem::path copy = std::filesystem::path(ra2::save_directory) / original.filename();
        std::error_code error;
        if (!std::filesystem::exists(copy, error))
        {
            if (!std::filesystem::copy_file(original, copy, error))
            {
                return false;
            }
            std::filesystem::permissions(
                copy, std::filesystem::perms::owner_write, std::filesystem::perm_options::add, error);
            created = true;
        }

        copyPath = copy.string();
        return true;
    }

    bool startsWith(const std::string &value, const std::string &prefix)
    {
        if (prefix.size() > value.size())
        {
            return false;
        }
        return std::equal(prefix.begin(), prefix.end(), value.begin());
    }

    void getLabelAndPath(const std::string &line, std::filesystem::path &path, std::string &label)
    {
        const size_t pos = line.find('|');
        if (pos == std::string::npos)
        {
            path = line;
            label = path.stem().string();
        }
        else
        {
            path = line.substr(0, pos);
            label = line.substr(pos + 1);
        }
    }

    void writeString(ra2::Buffer<char> &buffer, const std::string &value)
    {
        size_t const size = value.size();
        buffer.get<size_t>() = size;
        char *begin, *end;
        buffer.get(size, begin, end);
        memcpy(begin, value.data(), end - begin);
    }

    void readString(ra2::Buffer<const char> &buffer, std::string &value)
    {
        size_t const size = buffer.get<size_t const>();
        char const *begin, *end;
        buffer.get(size, begin, end);
        value.assign(begin, end);
    }

} // namespace

namespace ra2
{

    DiskControl::DiskControl()
        : myEjected{true, true}
        , myIndex{0, 0}
    {
    }

    const std::string &DiskControl::getCurrentDiskFolder() const
    {
        return myCurrentDiskFolder;
    }

    Drive_e DiskControl::activeDrive() const
    {
        return ra2::getDiskControlDrive();
    }

    void DiskControl::storeCurrentDiskFolder(const std::string &path)
    {
        // a bit of a workaround to a save state issue, where the disk folder is lost
        // this will only support 1 disk
        const std::filesystem::path filePath(path);
        myCurrentDiskFolder = filePath.parent_path().string();
    }

    bool DiskControl::insertDisk(const Drive_e drive, const std::string &path)
    {
        const bool writeProtected = gameDiskWriteProtection();
        const bool createIfNecessary = IMAGE_DONT_CREATE;

        if (insertFloppyDisk(drive, path, writeProtected, createIfNecessary))
        {
            myIndex[drive] = 0;
            myImages.clear();

            const std::filesystem::path filePath(path);
            myImages.push_back({filePath.string(), filePath.stem().string(), writeProtected, createIfNecessary});
            myEjected[drive] = false;
            return true;
        }

        return insertHardDisk(path);
    }

    bool DiskControl::insertPlaylist(const std::string &path)
    {
        const std::filesystem::path playlistPath(path);
        std::ifstream playlist(playlistPath);
        if (!playlist)
        {
            return false;
        }

        myImages.clear();
        const std::filesystem::path parent = playlistPath.parent_path();
        const std::filesystem::path savePath(ra2::save_directory);
        const std::string playlistStem = playlistPath.stem().string();

        std::string line;
        std::string pendingLabel;
        while (std::getline(playlist, line))
        {
            // should we trim initial spaces?
            if (startsWith(line, M3U_SAVEDISK))
            {
                pendingLabel.clear();
                const size_t index = myImages.size() + 1;
                const std::string filename = StrFormat("%s.save%" SIZE_T_FMT ".dsk", playlistStem.c_str(), index);

                // 1) use the label from #SAVEDISK:label
                std::string labelSuffix = line.substr(M3U_SAVEDISK.size());
                if (labelSuffix.empty())
                {
                    // 2) use an automatic label "the index"
                    labelSuffix = std::to_string(index);
                }

                // Always prefix with "Save Disk"
                const std::string label = SAVEDISK_LABEL + labelSuffix;

                const std::filesystem::path imagePath = savePath / filename;

                // TODO: this disk is NOT formatted
                myImages.push_back(
                    {imagePath.string(), label, IMAGE_USE_FILES_WRITE_PROTECT_STATUS, IMAGE_CREATE, true});
            }
            else if (startsWith(line, M3U_LABEL))
            {
                pendingLabel = line.substr(M3U_LABEL.size());
            }
            else if (startsWith(line, M3U_EXTINF))
            {
                // #EXTINF: standard - label follows the first comma
                const size_t comma = line.find(',', M3U_EXTINF.size());
                if (comma != std::string::npos)
                {
                    pendingLabel = line.substr(comma + 1);
                }
            }
            else if (!startsWith(line, M3U_COMMENT))
            {
                std::filesystem::path imagePath;
                std::string label;
                getLabelAndPath(line, imagePath, label);

                // #LABEL: or #EXTINF: override pipe label
                if (!pendingLabel.empty())
                {
                    label = pendingLabel;
                    pendingLabel.clear();
                }

                if (imagePath.is_relative())
                {
                    imagePath = parent / imagePath;
                }
                myImages.push_back({imagePath.string(), label, gameDiskWriteProtection(), IMAGE_DONT_CREATE, false});
            }
        }

        // insert the first image by default
        myIndex[DRIVE_1] = 0;

        const PlaylistStartDisk playlistStartDisk = getPlaylistStartDisk();

        if (playlistStartDisk == PlaylistStartDisk::Previous)
        {
            // if we have an initial disk image, let's try to honour it
            if (!ourInitialPath.empty() && ourInitialIndex < myImages.size() &&
                myImages[ourInitialIndex].path == ourInitialPath)
            {
                myIndex[DRIVE_1] = ourInitialIndex;
                // do we need to reset for next time?
                ourInitialPath.clear();
                ourInitialIndex = 0;
            }
        }

        // this is safe even if myImages is empty
        myEjected[DRIVE_1] = true;
        const bool inserted = setEjectedStateForDrive(DRIVE_1, false);

        // MultiDrive support: if M3U filename constains "(MD") or option is enabled, insert second disk into DRIVE_2
        // Only when starting from disk 0 to avoid conflicts with Previous resume
        // Skip save disks - those are for manual swap only
        if (inserted && myIndex[DRIVE_1] == 0 && myImages.size() > 1 &&
            (playlistStem.find("(MD") != std::string::npos || ra2::getFloppyMultiDrive()))
        {
            const auto &image = myImages[1];
            if (!image.isSaveDisk)
            {
                insertFloppyDisk(DRIVE_2, image.path, image.writeProtected, image.createIfNecessary);
                myEjected[DRIVE_2] = false;
                myIndex[DRIVE_2] = 1;
            }
        }

        return inserted;
    }

    bool DiskControl::insertFloppyDisk(
        const Drive_e drive, const std::string &path, const bool writeProtected, bool const createIfNecessary)
    {
        CardManager &cardManager = GetCardMgr();
        Disk2InterfaceCard *disk2Card = dynamic_cast<Disk2InterfaceCard *>(cardManager.GetObj(SLOT6));

        if (disk2Card)
        {
            std::string diskPath = path;
            bool diskWriteProtected = writeProtected;
            bool created = false;
            // save disks are created in the save folder already
            if (!writeProtected && !createIfNecessary && !copyToSaveFolder(path, diskPath, created))
            {
                diskWriteProtected = IMAGE_FORCE_WRITE_PROTECTED;
            }

            const ImageError_e error = disk2Card->InsertDisk(drive, diskPath, diskWriteProtected, createIfNecessary);
            const bool result = (error == eIMAGE_ERROR_NONE);

            ra2::log_cb(RETRO_LOG_INFO, "Insert into drive %d: %s -> %d\n", drive + 1, diskPath.c_str(), result);

            if (result)
            {
                storeCurrentDiskFolder(diskPath);
                return true;
            }

            if (created)
            {
                // not a floppy image, so no copy is needed
                std::error_code removeError;
                std::filesystem::remove(diskPath, removeError);
            }
        }

        return false;
    }

    bool DiskControl::insertHardDisk(const std::string &path)
    {
        CardManager &cardManager = GetCardMgr();
        HarddiskInterfaceCard *harddiskCard = dynamic_cast<HarddiskInterfaceCard *>(cardManager.GetObj(SLOT7));

        if (harddiskCard)
        {
            const bool ok = harddiskCard->Insert(HARDDISK_1, path);
            if (ok)
            {
                storeCurrentDiskFolder(path);
            }
            return ok;
        }

        return false;
    }

    bool DiskControl::getEjectedState() const
    {
        const Drive_e drive = activeDrive();
        return myEjected[drive];
    }

    bool DiskControl::setEjectedState(bool ejected)
    {
        const Drive_e drive = activeDrive();
        return setEjectedStateForDrive(drive, ejected);
    }

    bool DiskControl::setEjectedStateForDrive(const Drive_e drive, bool ejected)
    {
        if (myEjected[drive] == ejected)
        {
            return true;
        }

        CardManager &cardManager = GetCardMgr();
        Disk2InterfaceCard *disk2Card = dynamic_cast<Disk2InterfaceCard *>(cardManager.GetObj(SLOT6));

        bool result = false;
        if (disk2Card && myIndex[drive] < myImages.size())
        {
            if (ejected)
            {
                disk2Card->EjectDisk(drive);
                result = true;
                myEjected[drive] = ejected;
            }
            else
            {
                // if the other dirve has the same disk, eject it first to avoid conflicts
                const Drive_e otherDrive = (drive == DRIVE_1) ? DRIVE_2 : DRIVE_1;
                if (!myEjected[otherDrive] && (myIndex[otherDrive] == myIndex[drive]))
                {
                    disk2Card->EjectDisk(otherDrive);
                    myEjected[otherDrive] = true;
                    ra2::log_cb(
                        RETRO_LOG_INFO, "Ejecting drive %d to avoid conflict with drive %d\n", otherDrive + 1,
                        drive + 1);
                }
                const auto &image = myImages[myIndex[drive]];
                // inserted
                result = insertFloppyDisk(drive, image.path, image.writeProtected, image.createIfNecessary);
                myEjected[drive] = !result;
            }
        }

        return result;
    }

    size_t DiskControl::getImageIndex() const
    {
        const Drive_e drive = activeDrive();
        return myIndex[drive];
    }

    bool DiskControl::setImageIndex(size_t index)
    {
        const Drive_e drive = activeDrive();
        if (myEjected[drive])
        {
            myIndex[drive] = index;
            return true;
        }
        else
        {
            return false;
        }
    }

    size_t DiskControl::getNumImages() const
    {
        return myImages.size();
    }

    bool DiskControl::replaceImageIndex(size_t index, const std::string &path)
    {
        const Drive_e drive = activeDrive();
        if (myEjected[drive] && myIndex[drive] < myImages.size())
        {
            const std::filesystem::path filePath(path);

            myImages[index].path = filePath.string();
            myImages[index].label = filePath.stem().string();
            myImages[index].writeProtected = gameDiskWriteProtection();
            myImages[index].createIfNecessary = IMAGE_DONT_CREATE;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DiskControl::removeImageIndex(size_t index)
    {
        const Drive_e drive = activeDrive();
        if (myEjected[drive] && myIndex[drive] < myImages.size())
        {
            myImages.erase(myImages.begin() + index);
            if (myImages.empty() || myIndex[drive] == index)
            {
                myIndex[drive] = myImages.size();
            }
            else if (myIndex[drive] > index)
            {
                --myIndex[drive];
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DiskControl::addImageIndex()
    {
        myImages.push_back({"", "", IMAGE_FORCE_WRITE_PROTECTED, IMAGE_DONT_CREATE});
        return true;
    }

    bool DiskControl::getImagePath(unsigned index, char *path, size_t len) const
    {
        if (index < myImages.size())
        {
            strncpy(path, myImages[index].path.c_str(), len);
            path[len - 1] = 0;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DiskControl::getImageLabel(unsigned index, char *label, size_t len) const
    {
        if (index < myImages.size())
        {
            const Drive_e drive = activeDrive();
            const Drive_e otherDrive = (drive == DRIVE_1) ? DRIVE_2 : DRIVE_1;

            std::string displayLabel = "S6D" + std::to_string(drive + 1) + " - " + myImages[index].label;

            // Mark the label with an asterisk if the same disk is inserted in the other drive
            if (!myEjected[otherDrive] && myIndex[otherDrive] == index)
            {
                displayLabel += " <*>";
            }

            strncpy(label, displayLabel.c_str(), len);
            label[len - 1] = 0;
            return true;
        }
        else
        {
            return false;
        }
    }

    void DiskControl::serialise(Buffer<char> &buffer) const
    {
        writeString(buffer, myCurrentDiskFolder);
        for (int d = 0; d < NUM_DRIVES; ++d)
        {
            buffer.get<bool>() = myEjected[d];
            buffer.get<size_t>() = myIndex[d];
        }
        buffer.get<size_t>() = myImages.size();

        for (DiskInfo const &image : myImages)
        {
            writeString(buffer, image.path);
            writeString(buffer, image.label);
            buffer.get<bool>() = image.writeProtected;
            buffer.get<bool>() = image.createIfNecessary;
            buffer.get<bool>() = image.isSaveDisk;
        }
    }

    void DiskControl::deserialise(Buffer<char const> &buffer)
    {
        readString(buffer, myCurrentDiskFolder);
        for (int d = 0; d < NUM_DRIVES; ++d)
        {
            myEjected[d] = buffer.get<bool const>();
            myIndex[d] = buffer.get<size_t const>();
        }
        size_t const numberOfImages = buffer.get<size_t const>();
        myImages.clear();
        myImages.resize(numberOfImages);

        for (DiskInfo &image : myImages)
        {
            readString(buffer, image.path);
            readString(buffer, image.label);
            image.writeProtected = buffer.get<const bool>();
            image.createIfNecessary = buffer.get<const bool>();
            image.isSaveDisk = buffer.get<const bool>();
        }
    }

    unsigned DiskControl::ourInitialIndex = 0;
    std::string DiskControl::ourInitialPath;

    void DiskControl::setInitialPath(unsigned index, const char *path)
    {
        if (path && *path)
        {
            ourInitialIndex = index;
            ourInitialPath = path;
        }
    }

} // namespace ra2
