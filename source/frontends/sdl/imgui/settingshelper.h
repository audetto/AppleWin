#pragma once

#include "frontends/sdl/imgui/glselector.h"

#include "Card.h"
#include "CPU.h"
#include "Common.h"
#include "DiskImage.h"
#include "Video.h"
#include "CopyProtectionDongles.h"

#include <string>
#include <vector>
#include <map>

class FrameBase;

namespace sa2
{

    template <typename T, typename A>
    void comboIterator1(const char *label, const T current, const std::map<T, std::string> &values, const A action)
    {
        if (ImGui::BeginCombo(label, values.at(current).c_str()))
        {
            for (const auto &it : values)
            {
                // this only
                const T value = it.first; //
                const std::string &name = it.second;
                const bool isSelected = current == value;
                if (ImGui::Selectable(name.c_str(), isSelected))
                {
                    action(value);
                }
                if (isSelected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }

    template <typename T, typename A, typename F>
    void comboIterator2(
        const char *label, const T current, const std::vector<T> &values, const F getName, const A action)
    {
        if (ImGui::BeginCombo(label, getName(current).c_str()))
        {
            for (const auto &it : values)
            {
                const T value = it;
                const std::string &name = getName(it);
                const bool isSelected = current == value;
                if (ImGui::Selectable(name.c_str(), isSelected))
                {
                    action(value);
                }
                if (isSelected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }

    std::string getCardName(SS_CARDTYPE card);
    const std::string &getCPUName(eCpuType cpu);
    const std::string &getAppModeName(AppMode_e mode);
    const std::string &getDiskStatusName(Disk_Status_e status);

    std::vector<SS_CARDTYPE> getCardsForSlot(size_t slot);
    std::vector<SS_CARDTYPE> getExpansionCards();

    const std::map<eApple2Type, std::string> &getAapple2Types();
    const std::map<DONGLETYPE, std::string> &getDongleTypes();
    const std::map<VideoType_e, std::string> &getVideoTypes();

    void insertCard(size_t slot, SS_CARDTYPE card, FrameBase *frame);
    void setExpansionCard(SS_CARDTYPE card);
    void setVideoStyle(Video &video, const VideoStyle_e style, const bool enabled);

    void saveTFEEnabled(const int enabled);

    void changeBreakpoint(const uint32_t nAddress, const bool enableAndSet);

    ImVec4 colorrefToImVec4(const COLORREF cr);
    COLORREF imVec4ToColorref(const ImVec4 &color);

    char getPrintableChar(const uint8_t x); // copied from FormatCharTxtCtrl

    void sectableToClipboard(const std::string &text);

} // namespace sa2

namespace ImGui
{
    bool CheckBoxTristate(const char *label, int *v_tristate);
    void PushStyleCompact();
    void PopStyleCompact();
} // namespace ImGui
