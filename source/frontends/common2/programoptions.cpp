#include "frontends/common2/programoptions.h"
#include "linux/paddle.h"

#include "StdAfx.h"
#include "Log.h"
#include "Disk.h"
#include "Utilities.h"
#include "Core.h"
#include "Speaker.h"
#include "Riff.h"
#include "CardManager.h"

namespace common2
{

    EmulatorOptions::EmulatorOptions()
    {
        memclear = g_nMemoryClearType;
    }

    void applyOptions(const EmulatorOptions &options)
    {
        g_nMemoryClearType = options.memclear;
        g_bDisableDirectSound = options.noAudio;
        g_bDisableDirectSoundMockingboard = options.noAudio;

        bool bBoot = false;
        CardManager &cardManager = GetCardMgr();

        bool insertDisk2Card = false;
        LPCSTR szImageName_drive[NUM_DRIVES] = {};

        if (!options.disk1.empty())
        {
            insertDisk2Card = true;
            szImageName_drive[DRIVE_1] = options.disk1.c_str();
        }

        if (!options.disk2.empty())
        {
            insertDisk2Card = true;
            szImageName_drive[DRIVE_2] = options.disk2.c_str();
        }

        if (insertDisk2Card)
        {
            if (cardManager.QuerySlot(SLOT6) != CT_Disk2)
            {
                cardManager.Insert(SLOT6, CT_Disk2);
            }

            bool driveConnected[NUM_DRIVES] = {true, true};
            InsertFloppyDisks(SLOT6, szImageName_drive, driveConnected, bBoot);
        }

        bool insertHardDiskCard = false;
        LPCSTR szImageName_harddisk[NUM_HARDDISKS] = {};

        if (!options.hardDisk1.empty())
        {
            insertHardDiskCard = true;
            szImageName_harddisk[DRIVE_1] = options.hardDisk1.c_str();
        }

        if (!options.hardDisk2.empty())
        {
            insertHardDiskCard = true;
            szImageName_harddisk[DRIVE_2] = options.hardDisk2.c_str();
        }

        if (insertHardDiskCard)
        {
            if (cardManager.QuerySlot(SLOT7) != CT_GenericHDD)
            {
                cardManager.Insert(SLOT7, CT_GenericHDD);
            }

            InsertHardDisks(SLOT7, szImageName_harddisk, bBoot);
        }

        if (!options.customRom.empty())
        {
            CloseHandle(g_hCustomRom);
            g_hCustomRom = CreateFile(
                options.customRom.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, nullptr);
            if (g_hCustomRom == INVALID_HANDLE_VALUE)
            {
                LogFileOutput("Init: Failed to load Custom ROM: %s\n", options.customRom.c_str());
            }
        }

        if (!options.customRomF8.empty())
        {
            CloseHandle(g_hCustomRomF8);
            g_hCustomRomF8 = CreateFile(
                options.customRomF8.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, nullptr);
            if (g_hCustomRomF8 == INVALID_HANDLE_VALUE)
            {
                LogFileOutput("Init: Failed to load custom F8 ROM: %s\n", options.customRomF8.c_str());
            }
        }

        if (!options.wavFileSpeaker.empty())
        {
            if (RiffInitWriteFile(options.wavFileSpeaker.c_str(), SPKR_SAMPLE_RATE, 1))
            {
                Spkr_OutputToRiff();
            }
        }
        else if (!options.wavFileMockingboard.empty())
        {
            if (RiffInitWriteFile(options.wavFileMockingboard.c_str(), MockingboardCard::SAMPLE_RATE, 2))
            {
                GetCardMgr().GetMockingboardCardMgr().OutputToRiff();
            }
        }

        Paddle::setSquaring(options.paddleSquaring);
    }

} // namespace common2
