#include "StdAfx.h"

#include "SerialComms.h"
#include "CPU.h"
#include "Interface.h"
#include "Log.h"
#include "Memory.h"
#include "Registry.h"
#include "StrFormat.h"
#include "YamlHelper.h"

#include "../../../resource/resource.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#define TCP_SERIAL_PORT 1977

namespace
{
    constexpr UINT kBaud110 = 110;
    constexpr UINT kBaud300 = 300;
    constexpr UINT kBaud600 = 600;
    constexpr UINT kBaud1200 = 1200;
    constexpr UINT kBaud2400 = 2400;
    constexpr UINT kBaud4800 = 4800;
    constexpr UINT kBaud9600 = 9600;
    constexpr UINT kBaud19200 = 19200;
    constexpr UINT kBaud115200 = 115200;

    constexpr UINT kNoParity = 0;
    constexpr UINT kOddParity = 1;
    constexpr UINT kEvenParity = 2;
    constexpr UINT kMarkParity = 3;
    constexpr UINT kSpaceParity = 4;

    constexpr UINT kOneStopBit = 1;
    constexpr UINT kOne5StopBits = 15;
    constexpr UINT kTwoStopBits = 2;

    constexpr UINT kDtrDisable = 0;
    constexpr UINT kDtrEnable = 1;
    constexpr UINT kRtsDisable = 0;
    constexpr UINT kRtsEnable = 1;

    constexpr DWORD kMsCtsOn = 0x10;
    constexpr DWORD kMsDsrOn = 0x20;
    constexpr DWORD kMsRlsdOn = 0x80;

    struct SerialFdHandle : public CHANDLE
    {
        explicit SerialFdHandle(int fileDescriptor)
            : fd(fileDescriptor)
        {
        }

        ~SerialFdHandle() override
        {
            if (fd >= 0)
            {
                close(fd);
            }
        }

        int fd = -1;
    };

    int getSerialFd(HANDLE handle)
    {
        if (!handle || handle == INVALID_HANDLE_VALUE)
        {
            return -1;
        }

        SerialFdHandle *serialHandle = dynamic_cast<SerialFdHandle *>(handle);
        return serialHandle ? serialHandle->fd : -1;
    }

    bool stringEqualsNoCase(const std::string &lhs, const char *rhs)
    {
        if (!rhs || lhs.size() != strlen(rhs))
        {
            return false;
        }

        for (size_t i = 0; i < lhs.size(); ++i)
        {
            const char a = lhs[i];
            const char b = rhs[i];
            if (tolower(a) != tolower(b))
            {
                return false;
            }
        }

        return true;
    }

    speed_t mapBaud(UINT baud)
    {
        switch (baud)
        {
        case kBaud110:
            return B110;
        case kBaud300:
            return B300;
        case kBaud600:
            return B600;
        case kBaud1200:
            return B1200;
        case kBaud2400:
            return B2400;
        case kBaud4800:
            return B4800;
        case kBaud9600:
            return B9600;
        case kBaud19200:
            return B19200;
        case kBaud115200:
            return B115200;
        default:
            return B9600;
        }
    }
}

const UINT CSuperSerialCard::SERIALPORTITEM_INVALID_COM_PORT = 0;

// Default: 9600-8-N-1
SSC_DIPSW CSuperSerialCard::m_DIPSWDefault = {
    kBaud9600,
    FWMODE_CIC,

    kOneStopBit,
    8,
    kNoParity,
    false,
    true,
};

// 6551 ACIA Command Register bits
enum
{
    CMD_PARITY_MASK = 3 << 6,
    CMD_PARITY_ODD = 0 << 6,
    CMD_PARITY_EVEN = 1 << 6,
    CMD_PARITY_MARK = 2 << 6,
    CMD_PARITY_SPACE = 3 << 6,
    CMD_PARITY_ENA = 1 << 5,
    CMD_ECHO_MODE = 1 << 4,
    CMD_TX_MASK = 3 << 2,
    CMD_TX_IRQ_DIS_RTS_HIGH = 0 << 2,
    CMD_TX_IRQ_ENA_RTS_LOW = 1 << 2,
    CMD_TX_IRQ_DIS_RTS_LOW = 2 << 2,
    CMD_TX_IRQ_DIS_RTS_LOW_BRK = 3 << 2,
    CMD_RX_IRQ_DIS = 1 << 1,
    CMD_DTR = 1 << 0,
};

// 6551 ACIA Status Register bits
enum
{
    ST_IRQ = 1 << 7,
    ST_DSR = 1 << 6,
    ST_DCD = 1 << 5,
    ST_TX_EMPTY = 1 << 4,
    ST_RX_FULL = 1 << 3,
};

CSuperSerialCard::CSuperSerialCard(UINT slot)
    : Card(CT_SSC, slot)
    , m_strSerialPortChoices(1, '\0')
    , m_uTCPChoiceItemIdx(0)
    , m_bCfgSupportDCD(false)
    , m_pExpansionRom(NULL)
    , m_hFrameWindow(NULL)
{
    if (m_slot != 2)
        ThrowErrorInvalidSlot();

    m_dwSerialPortItem = 0;

    m_hCommHandle = INVALID_HANDLE_VALUE;
    m_hCommListenSocket = INVALID_SOCKET;
    m_hCommAcceptSocket = INVALID_SOCKET;
    m_hCommThread = NULL;

    for (UINT i = 0; i < COMMEVT_MAX; i++)
        m_hCommEvent[i] = NULL;

    m_o = nullptr;

    InternalReset();

    const size_t serialChoiceItemLength = 256;
    char serialPortName[serialChoiceItemLength];
    std::string regSection = RegGetConfigSlotSection(m_slot);
    RegLoadString(regSection.c_str(), REGVALUE_SERIAL_PORT_NAME, TRUE, serialPortName, sizeof(serialPortName), "");

    SetSerialPortName(serialPortName);
}

CSuperSerialCard::~CSuperSerialCard()
{
    CloseComm();

    delete[] m_pExpansionRom;
    m_pExpansionRom = NULL;
}

void CSuperSerialCard::Update(const ULONG)
{
    if (IsActive())
    {
        CheckCommEvent(0);
    }
}

void CSuperSerialCard::InternalReset()
{
    GetDIPSW();
    UpdateCommandAndControlRegs(0, 0);

    m_vbTxIrqPending = false;
    m_vbRxIrqPending = false;
    m_vbTxEmpty = true;

    m_vuRxCurrBuffer = 0;
    m_qComSerialBuffer[0].clear();
    m_qComSerialBuffer[1].clear();
    m_qTcpSerialBuffer.clear();

    m_uDTR = kDtrDisable;
    m_uRTS = kRtsDisable;
    m_dwModemStatus = m_kDefaultModemStatus;
}

void CSuperSerialCard::GetDIPSW()
{
    SetDIPSWDefaults();
}

void CSuperSerialCard::SetDIPSWDefaults()
{
    m_DIPSWCurrent = m_DIPSWDefault;
}

UINT CSuperSerialCard::BaudRateToIndex(UINT uBaudRate)
{
    switch (uBaudRate)
    {
    case kBaud110:
        return 0x05;
    case kBaud300:
        return 0x06;
    case kBaud600:
        return 0x07;
    case kBaud1200:
        return 0x08;
    case kBaud2400:
        return 0x0A;
    case kBaud4800:
        return 0x0C;
    case kBaud9600:
        return 0x0E;
    case kBaud19200:
        return 0x0F;
    case kBaud115200:
        return 0x00;
    default:
        return BaudRateToIndex(m_kDefaultBaudRate);
    }
}

void CSuperSerialCard::UpdateCommState()
{
    const int fd = getSerialFd(m_hCommHandle);
    if (fd < 0)
    {
        return;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        return;
    }

    cfmakeraw(&options);

    speed_t speed = mapBaud(m_uBaudRate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
    switch (m_uByteSize)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    default:
        options.c_cflag |= CS8;
        break;
    }

    if (m_uParity == kOddParity || m_uParity == kEvenParity)
    {
        options.c_cflag |= PARENB;
        if (m_uParity == kOddParity)
        {
            options.c_cflag |= PARODD;
        }
    }

    if (m_uStopBits == kTwoStopBits)
    {
        options.c_cflag |= CSTOPB;
    }

    options.c_cflag |= (CLOCAL | CREAD);

    tcsetattr(fd, TCSANOW, &options);

    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0)
    {
        if (m_uDTR == kDtrEnable)
            status |= TIOCM_DTR;
        else
            status &= ~TIOCM_DTR;

        if (m_uRTS == kRtsEnable)
            status |= TIOCM_RTS;
        else
            status &= ~TIOCM_RTS;

        ioctl(fd, TIOCMSET, &status);
    }
}

bool CSuperSerialCard::CheckComm()
{
    if (IsActive())
        return true;

    if (m_currentSerialPortName.empty() || stringEqualsNoCase(m_currentSerialPortName, "none"))
        return false;

    if (stringEqualsNoCase(m_currentSerialPortName, TEXT_SERIAL_TCP))
    {
        m_hCommListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (m_hCommListenSocket == INVALID_SOCKET)
            return false;

        int one = 1;
        setsockopt(m_hCommListenSocket, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(TCP_SERIAL_PORT);
        address.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(m_hCommListenSocket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            close(m_hCommListenSocket);
            m_hCommListenSocket = INVALID_SOCKET;
            return false;
        }

        if (listen(m_hCommListenSocket, 1) < 0)
        {
            close(m_hCommListenSocket);
            m_hCommListenSocket = INVALID_SOCKET;
            return false;
        }

        const int flags = fcntl(m_hCommListenSocket, F_GETFL, 0);
        if (flags >= 0)
            fcntl(m_hCommListenSocket, F_SETFL, flags | O_NONBLOCK);
    }
    else
    {
        const int fd = open(m_currentSerialPortName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
        {
            return false;
        }

        m_hCommHandle = new SerialFdHandle(fd);
        UpdateCommState();

        int status = 0;
        if (ioctl(fd, TIOCMGET, &status) == 0)
        {
            m_dwModemStatus = 0;
            if (status & TIOCM_CTS)
                m_dwModemStatus |= kMsCtsOn;
            if (status & TIOCM_DSR)
                m_dwModemStatus |= kMsDsrOn;
#ifdef TIOCM_CAR
            if (status & TIOCM_CAR)
                m_dwModemStatus |= kMsRlsdOn;
#elif defined(TIOCM_CD)
            if (status & TIOCM_CD)
                m_dwModemStatus |= kMsRlsdOn;
#endif
        }
    }

    return IsActive();
}

void CSuperSerialCard::CloseComm()
{
    CommTcpSerialCleanup();

    if (m_hCommHandle != INVALID_HANDLE_VALUE)
        CloseHandle(m_hCommHandle);

    m_hCommHandle = INVALID_HANDLE_VALUE;
}

void CSuperSerialCard::CommTcpSerialCleanup()
{
    if (m_hCommListenSocket != INVALID_SOCKET)
    {
        close(m_hCommListenSocket);
        m_hCommListenSocket = INVALID_SOCKET;

        CommTcpSerialClose();
    }
}

void CSuperSerialCard::CommTcpSerialClose()
{
    if (m_hCommAcceptSocket != INVALID_SOCKET)
    {
        shutdown(m_hCommAcceptSocket, SHUT_RDWR);
        close(m_hCommAcceptSocket);
        m_hCommAcceptSocket = INVALID_SOCKET;
    }

    m_qTcpSerialBuffer.clear();
}

void CSuperSerialCard::CommTcpSerialAccept()
{
    if ((m_hCommListenSocket != INVALID_SOCKET) && (m_hCommAcceptSocket == INVALID_SOCKET))
    {
        m_hCommAcceptSocket = accept(m_hCommListenSocket, NULL, NULL);
        if (m_hCommAcceptSocket != INVALID_SOCKET)
        {
            const int flags = fcntl(m_hCommAcceptSocket, F_GETFL, 0);
            if (flags >= 0)
                fcntl(m_hCommAcceptSocket, F_SETFL, flags | O_NONBLOCK);
        }
    }
}

void CSuperSerialCard::CommTcpSerialReceive()
{
    if (m_hCommAcceptSocket == INVALID_SOCKET)
    {
        return;
    }

    char data[0x80];
    bool gotData = false;

    while (true)
    {
        const int received = recv(m_hCommAcceptSocket, data, sizeof(data), 0);
        if (received > 0)
        {
            gotData = true;
            for (int i = 0; i < received; i++)
            {
                m_qTcpSerialBuffer.push_back(data[i]);
            }
            continue;
        }

        if (received == 0)
        {
            CommTcpSerialClose();
            break;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            break;
        }

        CommTcpSerialClose();
        break;
    }

    if (gotData && m_bRxIrqEnabled && !m_qTcpSerialBuffer.empty())
    {
        CpuIrqAssert(IS_SSC);
        m_vbRxIrqPending = true;
    }
}

BYTE __stdcall CSuperSerialCard::SSC_IORead(WORD PC, WORD uAddr, BYTE bWrite, BYTE uValue, ULONG nExecutedCycles)
{
    UINT uSlot = ((uAddr & 0xff) >> 4) - 8;
    CSuperSerialCard *pSSC = (CSuperSerialCard *)MemGetSlotParameters(uSlot);

    switch (uAddr & 0xf)
    {
    case 0x1:
    case 0x2:
        return pSSC->CommDipSw(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0x8:
        return pSSC->CommReceive(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0x9:
        return pSSC->CommStatus(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0xA:
        return pSSC->CommCommand(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0xB:
        return pSSC->CommControl(PC, uAddr, bWrite, uValue, nExecutedCycles);
    default:
        return IO_Null(PC, uAddr, bWrite, uValue, nExecutedCycles);
    }
}

BYTE __stdcall CSuperSerialCard::SSC_IOWrite(WORD PC, WORD uAddr, BYTE bWrite, BYTE uValue, ULONG nExecutedCycles)
{
    UINT uSlot = ((uAddr & 0xff) >> 4) - 8;
    CSuperSerialCard *pSSC = (CSuperSerialCard *)MemGetSlotParameters(uSlot);

    switch (uAddr & 0xf)
    {
    case 0x8:
        return pSSC->CommTransmit(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0x9:
        return pSSC->CommProgramReset(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0xA:
        return pSSC->CommCommand(PC, uAddr, bWrite, uValue, nExecutedCycles);
    case 0xB:
        return pSSC->CommControl(PC, uAddr, bWrite, uValue, nExecutedCycles);
    default:
        return IO_Null(PC, uAddr, bWrite, uValue, nExecutedCycles);
    }
}

BYTE __stdcall CSuperSerialCard::CommProgramReset(WORD, WORD, BYTE, BYTE, ULONG)
{
    UpdateCommandReg(m_uCommandByte & (CMD_PARITY_MASK | CMD_PARITY_ENA));
    return 0;
}

void CSuperSerialCard::UpdateCommandAndControlRegs(BYTE command, BYTE control)
{
    UpdateCommandReg(command);
    UpdateControlReg(control);
}

void CSuperSerialCard::UpdateCommandReg(BYTE command)
{
    m_uCommandByte = command;

    if (m_uCommandByte & CMD_PARITY_ENA)
    {
        switch (m_uCommandByte & CMD_PARITY_MASK)
        {
        case CMD_PARITY_ODD:
            m_uParity = kOddParity;
            break;
        case CMD_PARITY_EVEN:
            m_uParity = kEvenParity;
            break;
        case CMD_PARITY_MARK:
            m_uParity = kMarkParity;
            break;
        case CMD_PARITY_SPACE:
            m_uParity = kSpaceParity;
            break;
        }
    }
    else
    {
        m_uParity = kNoParity;
    }

    switch (m_uCommandByte & CMD_TX_MASK)
    {
    case CMD_TX_IRQ_DIS_RTS_HIGH:
        m_uRTS = kRtsDisable;
        break;
    default:
        m_uRTS = kRtsEnable;
        break;
    }

    if (m_DIPSWCurrent.bInterrupts && (m_uCommandByte & CMD_DTR))
    {
        m_bTxIrqEnabled = (m_uCommandByte & CMD_TX_MASK) == CMD_TX_IRQ_ENA_RTS_LOW;
        m_bRxIrqEnabled = (m_uCommandByte & CMD_RX_IRQ_DIS) == 0;
    }
    else
    {
        m_bTxIrqEnabled = false;
        m_bRxIrqEnabled = false;
    }

    m_uDTR = (m_uCommandByte & CMD_DTR) ? kDtrEnable : kDtrDisable;
}

BYTE __stdcall CSuperSerialCard::CommCommand(WORD, WORD, BYTE write, BYTE value, ULONG)
{
    if (!CheckComm())
        return 0;

    if (write && (value != m_uCommandByte))
    {
        UpdateCommandReg(value);
        UpdateCommState();
    }

    return m_uCommandByte;
}

void CSuperSerialCard::UpdateControlReg(BYTE control)
{
    m_uControlByte = control;

    switch (m_uControlByte & 0x0F)
    {
    case 0x00:
        m_uBaudRate = kBaud115200;
        break;
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
        m_uBaudRate = kBaud110;
        break;
    case 0x06:
        m_uBaudRate = kBaud300;
        break;
    case 0x07:
        m_uBaudRate = kBaud600;
        break;
    case 0x08:
        m_uBaudRate = kBaud1200;
        break;
    case 0x09:
    case 0x0A:
        m_uBaudRate = kBaud2400;
        break;
    case 0x0B:
    case 0x0C:
        m_uBaudRate = kBaud4800;
        break;
    case 0x0D:
    case 0x0E:
        m_uBaudRate = kBaud9600;
        break;
    case 0x0F:
        m_uBaudRate = kBaud19200;
        break;
    }

    switch (m_uControlByte & 0x60)
    {
    case 0x00:
        m_uByteSize = 8;
        break;
    case 0x20:
        m_uByteSize = 7;
        break;
    case 0x40:
        m_uByteSize = 6;
        break;
    case 0x60:
        m_uByteSize = 5;
        break;
    }

    if (m_uControlByte & 0x80)
    {
        if ((m_uByteSize == 8) && (m_uParity != kNoParity))
            m_uStopBits = kOneStopBit;
        else if ((m_uByteSize == 5) && (m_uParity == kNoParity))
            m_uStopBits = kOne5StopBits;
        else
            m_uStopBits = kTwoStopBits;
    }
    else
    {
        m_uStopBits = kOneStopBit;
    }
}

BYTE __stdcall CSuperSerialCard::CommControl(WORD, WORD, BYTE write, BYTE value, ULONG)
{
    if (!CheckComm())
        return 0;

    if (write && (value != m_uControlByte))
    {
        UpdateControlReg(value);
        UpdateCommState();
    }

    return m_uControlByte;
}

void CSuperSerialCard::CheckCommEvent(DWORD)
{
    CommTcpSerialAccept();
    CommTcpSerialReceive();

    const int fd = getSerialFd(m_hCommHandle);
    if (fd < 0)
    {
        return;
    }

    char data[0x100];
    bool gotData = false;

    while (true)
    {
        const ssize_t received = read(fd, data, sizeof(data));
        if (received > 0)
        {
            gotData = true;
            for (ssize_t i = 0; i < received; ++i)
            {
                m_qComSerialBuffer[0].push_back(static_cast<BYTE>(data[i]));
            }
            continue;
        }

        if (received == 0)
        {
            break;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            break;
        }

        CloseComm();
        break;
    }

    int modem = 0;
    if (ioctl(fd, TIOCMGET, &modem) == 0)
    {
        m_dwModemStatus = 0;
        if (modem & TIOCM_CTS)
            m_dwModemStatus |= kMsCtsOn;
        if (modem & TIOCM_DSR)
            m_dwModemStatus |= kMsDsrOn;
#ifdef TIOCM_CAR
        if (modem & TIOCM_CAR)
            m_dwModemStatus |= kMsRlsdOn;
#elif defined(TIOCM_CD)
        if (modem & TIOCM_CD)
            m_dwModemStatus |= kMsRlsdOn;
#endif
    }

    if (gotData && m_bRxIrqEnabled)
    {
        CpuIrqAssert(IS_SSC);
        m_vbRxIrqPending = true;
    }
}

BYTE __stdcall CSuperSerialCard::CommReceive(WORD, WORD, BYTE, BYTE, ULONG)
{
    if (!CheckComm())
        return 0;

    CheckCommEvent(0);

    if ((m_uCommandByte & CMD_DTR) == 0)
        return 0;

    BYTE result = 0;
    if (!m_qTcpSerialBuffer.empty())
    {
        result = m_qTcpSerialBuffer.front();
        m_qTcpSerialBuffer.pop_front();

        if (m_bRxIrqEnabled && !m_qTcpSerialBuffer.empty())
        {
            CpuIrqAssert(IS_SSC);
            m_vbRxIrqPending = true;
        }
        return result;
    }

    if (!m_qComSerialBuffer[0].empty())
    {
        result = m_qComSerialBuffer[0].front();
        m_qComSerialBuffer[0].pop_front();

        if (m_bRxIrqEnabled && !m_qComSerialBuffer[0].empty())
        {
            CpuIrqAssert(IS_SSC);
            m_vbRxIrqPending = true;
        }
    }

    return result;
}

void CSuperSerialCard::TransmitDone(void)
{
    m_vbTxEmpty = true;

    if (m_bTxIrqEnabled)
    {
        CpuIrqAssert(IS_SSC);
        m_vbTxIrqPending = true;
    }
}

BYTE __stdcall CSuperSerialCard::CommTransmit(WORD, WORD, BYTE, BYTE value, ULONG)
{
    if (!CheckComm())
        return 0;

    if ((m_uCommandByte & CMD_TX_MASK) == CMD_TX_IRQ_DIS_RTS_HIGH)
        return 0;

    bool transmitted = false;

    if (m_hCommAcceptSocket != INVALID_SOCKET)
    {
        BYTE data = value;
        if (m_uByteSize < 8)
        {
            data &= ~(1 << m_uByteSize);
        }

        const int sent = send(m_hCommAcceptSocket, reinterpret_cast<const char *>(&data), 1, 0);
        transmitted = (sent == 1);
        if (!transmitted && sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            CommTcpSerialClose();
        }
    }
    else
    {
        const int fd = getSerialFd(m_hCommHandle);
        if (fd >= 0)
        {
            const ssize_t sent = write(fd, &value, 1);
            transmitted = (sent == 1);
        }
    }

    if (transmitted)
    {
        m_vbTxEmpty = false;
        TransmitDone();
    }

    return 0;
}

BYTE __stdcall CSuperSerialCard::CommStatus(WORD, WORD, BYTE, BYTE, ULONG)
{
    if (!CheckComm())
        return ST_DSR | ST_DCD | ST_TX_EMPTY;

    CheckCommEvent(0);

    DWORD modemStatus = m_kDefaultModemStatus;
    if (getSerialFd(m_hCommHandle) >= 0)
    {
        modemStatus = m_dwModemStatus;
        if (!m_bCfgSupportDCD)
        {
            modemStatus &= ~kMsRlsdOn;
            if (modemStatus & kMsDsrOn)
                modemStatus |= kMsRlsdOn;
        }
    }
    else if (m_hCommListenSocket != INVALID_SOCKET && m_hCommAcceptSocket != INVALID_SOCKET)
    {
        modemStatus = kMsRlsdOn | kMsDsrOn | kMsCtsOn;
    }

    BYTE IRQ = 0;
    if (m_bTxIrqEnabled)
    {
        IRQ |= m_vbTxIrqPending ? ST_IRQ : 0;
        m_vbTxIrqPending = false;
    }
    if (m_bRxIrqEnabled)
    {
        IRQ |= m_vbRxIrqPending ? ST_IRQ : 0;
        m_vbRxIrqPending = false;
    }

    BYTE DSR = (modemStatus & kMsDsrOn) ? 0x00 : ST_DSR;
    BYTE DCD = (modemStatus & kMsRlsdOn) ? 0x00 : ST_DCD;
    BYTE TX_EMPTY = m_vbTxEmpty ? ST_TX_EMPTY : 0;
    BYTE RX_FULL = (!m_qComSerialBuffer[0].empty() || !m_qTcpSerialBuffer.empty()) ? ST_RX_FULL : 0;

    CpuIrqDeassert(IS_SSC);

    return IRQ | DSR | DCD | TX_EMPTY | RX_FULL;
}

BYTE __stdcall CSuperSerialCard::CommDipSw(WORD, WORD addr, BYTE, BYTE, ULONG)
{
    BYTE sw = 0;

    switch (addr & 0xf)
    {
    case 1:
        sw = (BaudRateToIndex(m_DIPSWCurrent.uBaudRate) << 4) | m_DIPSWCurrent.eFirmwareMode;
        break;

    case 2:
    {
        BYTE SW2_1 = m_DIPSWCurrent.uStopBits == kTwoStopBits ? 1 : 0;
        BYTE SW2_2 = m_DIPSWCurrent.uByteSize == 7 ? 1 : 0;

        BYTE SW2_3 = 0;
        BYTE SW2_4 = 0;
        switch (m_DIPSWCurrent.uParity)
        {
        case kOddParity:
            SW2_3 = 0;
            SW2_4 = 1;
            break;
        case kEvenParity:
            SW2_3 = 1;
            SW2_4 = 1;
            break;
        default:
            SW2_3 = 0;
            SW2_4 = 0;
            break;
        }

        BYTE SW2_5 = m_DIPSWCurrent.bLinefeed ? 0 : 1;
        BYTE CTS = 1;

        const int fd = getSerialFd(m_hCommHandle);
        if (CheckComm() && fd >= 0)
        {
            int modem = 0;
            if (ioctl(fd, TIOCMGET, &modem) == 0)
            {
                CTS = (modem & TIOCM_CTS) ? 0 : 1;
            }
        }
        else if (m_hCommListenSocket != INVALID_SOCKET)
        {
            CTS = (m_hCommAcceptSocket != INVALID_SOCKET) ? 0 : 1;
        }

        sw = (SW2_1 << 7) | (0 << 6) | (SW2_2 << 5) | (0 << 4) | (SW2_3 << 3) | (SW2_4 << 2) | (SW2_5 << 1) |
             (CTS << 0);
        break;
    }
    }

    return sw;
}

void CSuperSerialCard::InitializeIO(LPBYTE pCxRomPeripheral)
{
    const UINT SSC_FW_SIZE = 2 * 1024;
    const UINT SSC_SLOT_FW_SIZE = 256;
    const UINT SSC_SLOT_FW_OFFSET = 7 * 256;

    BYTE *pData = GetFrame().GetResource(IDR_SSC_FW, "FIRMWARE", SSC_FW_SIZE);
    if (pData == NULL)
        return;

    memcpy(pCxRomPeripheral + m_slot * SSC_SLOT_FW_SIZE, pData + SSC_SLOT_FW_OFFSET, SSC_SLOT_FW_SIZE);

    if (m_pExpansionRom == NULL)
    {
        m_pExpansionRom = new BYTE[SSC_FW_SIZE];
        memcpy(m_pExpansionRom, pData, SSC_FW_SIZE);
    }

    RegisterIoHandler(m_slot, &CSuperSerialCard::SSC_IORead, &CSuperSerialCard::SSC_IOWrite, NULL, NULL, this, m_pExpansionRom);
}

void CSuperSerialCard::Reset(const bool)
{
    CloseComm();
    InternalReset();
}

void CSuperSerialCard::ScanCOMPorts()
{
    m_vecSerialPortsItems.clear();
    m_vecSerialPortsItems.push_back(SERIALPORTITEM_INVALID_COM_PORT); // None
    m_vecSerialPortsItems.push_back(SERIALPORTITEM_INVALID_COM_PORT); // TCP
    m_uTCPChoiceItemIdx = 1;
}

std::string const &CSuperSerialCard::GetSerialPortChoices()
{
    if (IsActive())
        return m_strSerialPortChoices;

    ScanCOMPorts();

    m_strSerialPortChoices = "None";
    m_strSerialPortChoices += '\0';
    m_strSerialPortChoices += "TCP";
    m_strSerialPortChoices += '\0';

    return m_strSerialPortChoices;
}

void CSuperSerialCard::CommSetSerialPort(DWORD dwNewSerialPortItem)
{
    if (m_dwSerialPortItem == dwNewSerialPortItem)
        return;

    if (IsActive())
        return;

    m_dwSerialPortItem = dwNewSerialPortItem;

    if (m_dwSerialPortItem == m_uTCPChoiceItemIdx)
    {
        m_currentSerialPortName = TEXT_SERIAL_TCP;
    }
    else
    {
        m_currentSerialPortName.clear();
    }

    SetRegistrySerialPortName();
}

void CSuperSerialCard::SetSerialPortName(const char *pSerialPortName)
{
    m_currentSerialPortName = pSerialPortName ? pSerialPortName : "";

    if (m_currentSerialPortName.empty() || stringEqualsNoCase(m_currentSerialPortName, "none"))
    {
        m_currentSerialPortName.clear();
        m_dwSerialPortItem = 0;
    }
    else if (stringEqualsNoCase(m_currentSerialPortName, TEXT_SERIAL_TCP))
    {
        ScanCOMPorts();
        m_dwSerialPortItem = m_uTCPChoiceItemIdx;
    }
    else
    {
        // Manual Linux serial path, eg. /dev/ttyUSB0.
        m_dwSerialPortItem = 0;
    }
}

void CSuperSerialCard::SetRegistrySerialPortName(void)
{
    std::string regSection = RegGetConfigSlotSection(m_slot);
    RegSaveString(regSection.c_str(), REGVALUE_SERIAL_PORT_NAME, TRUE, GetSerialPortName());
}

bool CSuperSerialCard::CommThInit()
{
    return true;
}

void CSuperSerialCard::CommThUninit()
{
}

DWORD WINAPI CSuperSerialCard::CommThread(LPVOID)
{
    return 0;
}

// Unit version history:
// 2: Added: Support DCD flag
//    Removed: redundant data (encapsulated in Command & Control bytes)
static const UINT kUNIT_VERSION = 2;

#define SS_YAML_VALUE_CARD_SSC "Super Serial Card"

#define SS_YAML_KEY_DIPSWDEFAULT "DIPSW Default"
#define SS_YAML_KEY_DIPSWCURRENT "DIPSW Current"

#define SS_YAML_KEY_BAUDRATE "Baud Rate"
#define SS_YAML_KEY_FWMODE "Firmware mode"
#define SS_YAML_KEY_STOPBITS "Stop Bits"
#define SS_YAML_KEY_BYTESIZE "Byte Size"
#define SS_YAML_KEY_PARITY "Parity"
#define SS_YAML_KEY_LINEFEED "Linefeed"
#define SS_YAML_KEY_INTERRUPTS "Interrupts"
#define SS_YAML_KEY_CONTROL "Control Byte"
#define SS_YAML_KEY_COMMAND "Command Byte"
#define SS_YAML_KEY_INACTIVITY "Comm Inactivity"
#define SS_YAML_KEY_TXIRQENABLED "TX IRQ Enabled"
#define SS_YAML_KEY_RXIRQENABLED "RX IRQ Enabled"
#define SS_YAML_KEY_TXIRQPENDING "TX IRQ Pending"
#define SS_YAML_KEY_RXIRQPENDING "RX IRQ Pending"
#define SS_YAML_KEY_WRITTENTX "Written TX"
#define SS_YAML_KEY_SERIALPORTNAME "Serial Port Name"
#define SS_YAML_KEY_SUPPORT_DCD "Support DCD"

const std::string &CSuperSerialCard::GetSnapshotCardName(void)
{
    static const std::string name(SS_YAML_VALUE_CARD_SSC);
    return name;
}

void CSuperSerialCard::SaveSnapshotDIPSW(YamlSaveHelper &yamlSaveHelper, std::string key, SSC_DIPSW &dipsw)
{
    YamlSaveHelper::Label label(yamlSaveHelper, "%s:\n", key.c_str());
    yamlSaveHelper.SaveUint(SS_YAML_KEY_BAUDRATE, dipsw.uBaudRate);
    yamlSaveHelper.SaveUint(SS_YAML_KEY_FWMODE, dipsw.eFirmwareMode);
    yamlSaveHelper.SaveUint(SS_YAML_KEY_STOPBITS, dipsw.uStopBits);
    yamlSaveHelper.SaveUint(SS_YAML_KEY_BYTESIZE, dipsw.uByteSize);
    yamlSaveHelper.SaveUint(SS_YAML_KEY_PARITY, dipsw.uParity);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_LINEFEED, dipsw.bLinefeed);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_INTERRUPTS, dipsw.bInterrupts);
}

void CSuperSerialCard::SaveSnapshot(YamlSaveHelper &yamlSaveHelper)
{
    YamlSaveHelper::Slot slot(yamlSaveHelper, GetSnapshotCardName(), m_slot, kUNIT_VERSION);

    YamlSaveHelper::Label unit(yamlSaveHelper, "%s:\n", SS_YAML_KEY_STATE);
    SaveSnapshotDIPSW(yamlSaveHelper, SS_YAML_KEY_DIPSWDEFAULT, m_DIPSWDefault);
    SaveSnapshotDIPSW(yamlSaveHelper, SS_YAML_KEY_DIPSWCURRENT, m_DIPSWCurrent);
    yamlSaveHelper.SaveHexUint8(SS_YAML_KEY_CONTROL, m_uControlByte);
    yamlSaveHelper.SaveHexUint8(SS_YAML_KEY_COMMAND, m_uCommandByte);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_TXIRQPENDING, m_vbTxIrqPending);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_RXIRQPENDING, m_vbRxIrqPending);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_WRITTENTX, m_vbTxEmpty);
    yamlSaveHelper.SaveBool(SS_YAML_KEY_SUPPORT_DCD, m_bCfgSupportDCD);
    yamlSaveHelper.SaveString(SS_YAML_KEY_SERIALPORTNAME, GetSerialPortName());
}

void CSuperSerialCard::LoadSnapshotDIPSW(YamlLoadHelper &yamlLoadHelper, std::string key, SSC_DIPSW &dipsw)
{
    if (!yamlLoadHelper.GetSubMap(key))
        throw std::runtime_error("Card: Expected key: " + key);

    dipsw.uBaudRate = yamlLoadHelper.LoadUint(SS_YAML_KEY_BAUDRATE);
    dipsw.eFirmwareMode = (eFWMODE)yamlLoadHelper.LoadUint(SS_YAML_KEY_FWMODE);
    dipsw.uStopBits = yamlLoadHelper.LoadUint(SS_YAML_KEY_STOPBITS);
    dipsw.uByteSize = yamlLoadHelper.LoadUint(SS_YAML_KEY_BYTESIZE);
    dipsw.uParity = yamlLoadHelper.LoadUint(SS_YAML_KEY_PARITY);
    dipsw.bLinefeed = yamlLoadHelper.LoadBool(SS_YAML_KEY_LINEFEED);
    dipsw.bInterrupts = yamlLoadHelper.LoadBool(SS_YAML_KEY_INTERRUPTS);

    yamlLoadHelper.PopMap();
}

bool CSuperSerialCard::LoadSnapshot(YamlLoadHelper &yamlLoadHelper, UINT version)
{
    if (version < 1 || version > kUNIT_VERSION)
        ThrowErrorInvalidVersion(version);

    LoadSnapshotDIPSW(yamlLoadHelper, SS_YAML_KEY_DIPSWDEFAULT, m_DIPSWDefault);
    LoadSnapshotDIPSW(yamlLoadHelper, SS_YAML_KEY_DIPSWCURRENT, m_DIPSWCurrent);

    if (version == 1)
    {
        yamlLoadHelper.LoadUint(SS_YAML_KEY_PARITY);
        yamlLoadHelper.LoadBool(SS_YAML_KEY_TXIRQENABLED);
        yamlLoadHelper.LoadBool(SS_YAML_KEY_RXIRQENABLED);

        yamlLoadHelper.LoadUint(SS_YAML_KEY_BAUDRATE);
        yamlLoadHelper.LoadUint(SS_YAML_KEY_STOPBITS);
        yamlLoadHelper.LoadUint(SS_YAML_KEY_BYTESIZE);

        yamlLoadHelper.LoadUint(SS_YAML_KEY_INACTIVITY);
    }
    else if (version >= 2)
    {
        SupportDCD(yamlLoadHelper.LoadBool(SS_YAML_KEY_SUPPORT_DCD));
    }

    UINT uCommandByte = yamlLoadHelper.LoadUint(SS_YAML_KEY_COMMAND);
    UINT uControlByte = yamlLoadHelper.LoadUint(SS_YAML_KEY_CONTROL);
    UpdateCommandAndControlRegs(uCommandByte, uControlByte);

    m_vbTxIrqPending = yamlLoadHelper.LoadBool(SS_YAML_KEY_TXIRQPENDING);
    m_vbRxIrqPending = yamlLoadHelper.LoadBool(SS_YAML_KEY_RXIRQPENDING);
    m_vbTxEmpty = yamlLoadHelper.LoadBool(SS_YAML_KEY_WRITTENTX);

    if (m_vbTxIrqPending || m_vbRxIrqPending)
        CpuIrqAssert(IS_SSC);

    std::string serialPortName = yamlLoadHelper.LoadString(SS_YAML_KEY_SERIALPORTNAME);
    SetSerialPortName(serialPortName.c_str());
    SetRegistrySerialPortName();

    return true;
}
