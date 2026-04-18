#pragma once

#include <vector>
#include <cstdint>
#include <functional>
#include <optional>

class CassetteTape
{
public:
    typedef int8_t tape_data_t;

    void setData(const std::string &filename, const std::vector<tape_data_t> &data, const int frequency);
    BYTE getValue(const ULONG nExecutedCycles);

    struct TapeInfo
    {
        std::string_view filename;
        uint32_t duration; // ms
        uint32_t position; // ms
        double playbackRate; // seconds per second
        int frequency;
        uint8_t bit;
    };

    void getTapeInfo(TapeInfo &info) const;
    void eject();
    void rewind();

    std::function<void(double /* playbackRate */)> playbackRateChangeCallback = 0;

    static CassetteTape &instance();

private:
    BYTE getBitValue(const tape_data_t val);
    tape_data_t getCurrentWave(size_t &pos) const;

    std::vector<tape_data_t> myData;

    std::optional<int64_t> myBaseCycles;
    bool myReachedEnd = false;
    int myFrequency;
    BYTE myLastBit = 1;     // negative wave
    std::string myFilename; // just for info

    static constexpr tape_data_t myThreshold = 5;
};
