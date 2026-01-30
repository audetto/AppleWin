#include "StdAfx.h"

#include "cassettetape.h"

#include "Core.h"
#include "Tape.h"
#include "Memory.h"
#include "Pravets.h"
#include "CPU.h"

#include <limits>

CassetteTape &CassetteTape::instance()
{
    static CassetteTape tape;
    return tape;
}

void CassetteTape::setData(const std::string &filename, const std::vector<tape_data_t> &data, const int frequency)
{
    myFilename = filename;
    myData = data;
    myFrequency = frequency;
    rewind();
}

void CassetteTape::eject()
{
    rewind();
    myData.clear();
}

void CassetteTape::rewind()
{
    if (myBaseCycles > 0 && !myReachedEnd && playbackRateChangeCallback)
    {
        // rewind() called while playing, notify playback stop
        playbackRateChangeCallback(0);
    }
    myBaseCycles = -1;
    myReachedEnd = false;
}

BYTE CassetteTape::getBitValue(const tape_data_t val)
{
    // threshold not needed for https://asciiexpress.net
    // but probably necessary for a real audio file
    //
    // this has been tested on all formats from https://asciiexpress.net
    //
    // we are extracting the sign bit (set for negative numbers)
    // this is really important for the asymmetric wave in https://asciiexpress.net/diskserver/
    // not so much for the other cases
    if (val > myThreshold)
    {
        myLastBit = 0;
    }
    else if (val < -myThreshold)
    {
        myLastBit = 1;
    }
    // else leave it unchanged to the previous value
    return myLastBit;
}

CassetteTape::tape_data_t CassetteTape::getCurrentWave(size_t &pos) const
{
    if (myBaseCycles >= 0)
    {
        const double delta = g_nCumulativeCycles - myBaseCycles;
        const double position = delta / g_fCurrentCLK6502 * myFrequency;
        pos = static_cast<size_t>(position);

        if (pos + 1 < myData.size())
        {
            // linear interpolation
            const double reminder = position - pos;
            const double value = (1.0 - reminder) * myData[pos] + reminder * myData[pos + 1];
            return lround(value);
        }
        else
        {
            pos = myData.size() - 1;
        }
    }
    else
    {
        pos = 0;
    }
    return std::numeric_limits<tape_data_t>::min();
}

BYTE CassetteTape::getValue(const ULONG nExecutedCycles)
{
    CpuCalcCycles(nExecutedCycles);

    if (myBaseCycles < 0)
    {
        // start play as soon as TAPEIN is read
        myBaseCycles = g_nCumulativeCycles;
        if (playbackRateChangeCallback)
        {
            playbackRateChangeCallback(1);
        }
    }

    size_t pos;
    const tape_data_t val = getCurrentWave(pos);
    const BYTE highBit = getBitValue(val);

    if (pos > (myData.size() * 0.99) && !myReachedEnd)
    {
        // playback ended, notify playback stop
        // - getValue() isn't called all the way through pos == myData.size() - 1,
        //   presumably because it read all the data it cared about, so we fudge it
        //   at 99%.
        if (playbackRateChangeCallback)
        {
            playbackRateChangeCallback(0);
        }
        myReachedEnd = true; // avoid invoking callback again
    }

    return highBit;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void CassetteTape::getTapeInfo(TapeInfo &info) const
{
    info.filename = myFilename;
    size_t pos;
    const tape_data_t val = getCurrentWave(pos);
    const size_t size = myData.size();
    info.bit = myLastBit;
    info.duration = (size * 1000.0) / myFrequency;
    info.position = (pos * 1000.0) / myFrequency;
    info.playbackRate = (myBaseCycles >= 0 && pos < size - 1) ? 1 : 0;
    info.frequency = myFrequency;
}
#pragma GCC diagnostic pop

BYTE __stdcall TapeRead(WORD pc, WORD address, BYTE, BYTE, ULONG nExecutedCycles) // $C060 TAPEIN
{
    if (g_Apple2Type == A2TYPE_PRAVETS8A)
        return GetPravets().GetKeycode(MemReadFloatingBus(nExecutedCycles));

    const BYTE highBit = CassetteTape::instance().getValue(nExecutedCycles);

    return MemReadFloatingBus(highBit, nExecutedCycles);
}

BYTE __stdcall TapeWrite(WORD, WORD address, BYTE, BYTE, ULONG nExecutedCycles) // $C020 TAPEOUT
{
    return 0;
}
