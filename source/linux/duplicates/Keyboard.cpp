#include "StdAfx.h"
#include "Keyboard.h"

#include "Core.h"
#include "YamlHelper.h"

namespace
{
    bool g_bCapsLock = true; // Caps lock key for Apple2 and Lat/Cyr lock for Pravets8

    std::queue<BYTE> keys;
    BYTE keycode = 0;
    bool bKeyWasRead = false;
} // namespace

void addKeyToBuffer(BYTE key)
{
    // If the previous key was read by the CPU but not cleared (ignored),
    // we overwrite it (pop it) to prevent blocking the queue.
    if (bKeyWasRead && !keys.empty())
    {
        keys.pop();
        bKeyWasRead = false;
    }
    keys.push(key);
}

void addTextToBuffer(const char *text)
{
    while (*text)
    {
        if (*text == '\n')
        {
            addKeyToBuffer(0x0d);
        }
        else if (*text >= 0x20 && *text <= 0x7e)
        {
            addKeyToBuffer(*text);
        }
        // skip non ASCII characters
        ++text;
    }
}

bool KeybGetCapsStatus()
{
    return g_bCapsLock;
}

BYTE KeybGetKeycode()
{
    return keycode;
}

// $C000: read
BYTE KeybReadData()
{
    LogFileTimeUntilFirstKeyRead();

    if (!keys.empty())
    {
        keycode = keys.front();
        bKeyWasRead = true;
        return keycode | 0x80;
    }
    return keycode;
}

// $C010: read (!IS_APPLE2)
BYTE KeybReadFlag()
{
    _ASSERT(!IS_APPLE2); // And also not Pravets machines?

    KeybClearStrobe();

    // AKD
    return keycode | (keys.empty() ? 0 : 0x80);
}

// $C010: write (all) & read (IS_APPLE2)
BYTE KeybClearStrobe(void)
{
    if (keys.empty())
    {
        return 0;
    }
    else
    {
        keycode = keys.front();
        keys.pop();
        bKeyWasRead = false;
        return keycode | 0x80;
    }
}

void KeybReset()
{
    keycode = 0;
    std::queue<BYTE>().swap(keys);
    bKeyWasRead = false;
}

bool KeybGetShiftStatus()
{
    return false;
}

bool KeybGetAltStatus()
{
    return false;
}

bool KeybGetCtrlStatus()
{
    return false;
}

void KeybSetAltGrSendsWM_CHAR(bool state)
{
}

void KeybSetCapsLock(bool state)
{
}

#define SS_YAML_KEY_LASTKEY "Last Key"
#define SS_YAML_KEY_KEYWAITING "Key Waiting"

static std::string KeybGetSnapshotStructName(void)
{
    static const std::string name("Keyboard");
    return name;
}

void KeybSaveSnapshot(YamlSaveHelper &yamlSaveHelper)
{
    YamlSaveHelper::Label state(yamlSaveHelper, "%s:\n", KeybGetSnapshotStructName().c_str());
    yamlSaveHelper.SaveHexUint8(SS_YAML_KEY_LASTKEY, keys.empty() ? keycode : keys.front());
    yamlSaveHelper.SaveBool(SS_YAML_KEY_KEYWAITING, !keys.empty());
}

void KeybLoadSnapshot(YamlLoadHelper &yamlLoadHelper, UINT version)
{
    if (!yamlLoadHelper.GetSubMap(KeybGetSnapshotStructName()))
        return;

    KeybReset();

    keycode = (BYTE)yamlLoadHelper.LoadUint(SS_YAML_KEY_LASTKEY);
    if (version >= 2)
    {
        const bool keywaiting = yamlLoadHelper.LoadBool(SS_YAML_KEY_KEYWAITING);
        if (keywaiting)
        {
            addKeyToBuffer(keycode);
        }
    }

    yamlLoadHelper.PopMap();
}
