#include "StdAfx.h"
#include "frontends/sdl/monitor/binarymonitor.h"
#include "frontends/sdl/monitor/binarybuffer.h"
#include "frontends/sdl/monitor/payloadbuffer.h"
#include "frontends/sdl/monitor/breakpoints.h"
#include "frontends/sdl/monitor/exception.h"
#include "frontends/sdl/monitor/commands.h"

#include "frontends/common2/commonframe.h"

#include "linux/version.h"

#include "Utilities.h"
#include "Log.h"
#include "Core.h"
#include "Interface.h"
#include "CPU.h"
#include "Memory.h"
#include "CardManager.h"
#include "Disk.h"
#include "Debugger/Debug.h"
#include "Tfe/DNS.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>


#define MON_EVENT_ID 0xffffffff

#define LOG_COMMANDS

namespace
{

  void parseAddress(const std::string & address, sockaddr_in & server)
  {
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(6502);  // default

    const size_t posOfColon = address.find(':');
    const std::string name = address.substr(0, posOfColon);
    server.sin_addr.s_addr = getHostByName(name);

    if (posOfColon != std::string::npos)
    {
      const std::string port = address.substr(posOfColon + 1);
      if (!port.empty())
      {
        server.sin_port = htons(strtoul(port.c_str(), nullptr, 10));
      }
    }
  }

  std::map<uint8_t, std::string> getBankNames()
  {
    size_t id = 0;
    std::map<uint8_t, std::string> names;
    names[id++] = "default";
    names[id++] = "ram";
    names[id++] = "io";

    size_t i = 0;
    void * bank;
    while ((bank = MemGetBankPtr(i)))
    {
      const std::string name = "bank " + std::to_string(i);
      names[id++] = name;
      ++i;
    }

    return names;
  }

  bool isStopped(const AppMode_e mode)
  {
    switch (mode)
    {
    case MODE_RUNNING:
    case MODE_STEPPING:
      return false;
    case MODE_DEBUG:
    case MODE_PAUSED:
    default:
      return true;
    }
  }

}


namespace binarymonitor
{

  BinaryClient::BinaryClient(const int socket, common2::CommonFrame * frame)
    : myAvailableRegisters({
        {0, {"A",  sizeof(regs.a),  &regs.a}},
        {1, {"X",  sizeof(regs.x),  &regs.x}},
        {2, {"Y",  sizeof(regs.y),  &regs.y}},
        {3, {"PC", sizeof(regs.pc), &regs.pc}},
        {4, {"SP", sizeof(BYTE), &regs.sp}}, // this works on Little Endian CPUs.
        {5, {"PS", sizeof(regs.ps), &regs.ps}},
      })
    , myBankNames(getBankNames())
    , mySocket(socket)
    , myFrame(frame)
    , myStopped(isStopped(g_nAppMode))
  {
    reset();

  #ifdef LOG_COMMANDS
    LogOutput("New client: %d\n", mySocket);
  #endif
  }

  BinaryClient::~BinaryClient()
  {
  #ifdef LOG_COMMANDS
    LogOutput("Del client: %d\n", mySocket);
  #endif
    close(mySocket);
  }

  void BinaryClient::reset()
  {
    myCommandRead = 0;
    myPayloadRead = 0;
  }

  size_t BinaryClient::readData(char * dest, size_t len)
  {
    const ssize_t data = recv(mySocket, dest, len, MSG_DONTWAIT);
    if (data > 0)
    {
      return data;
    }
    else if (data == 0)
    {
      throw std::runtime_error("graceful termination");
    }
    else
    {
      throwIfError(data);
    }
    return 0;
  }

  void BinaryClient::throwIfError(const ssize_t result)
  {
    if (result < 0)
    {
      const int error = errno;
      if (error != EAGAIN && error != EWOULDBLOCK)
      {
        throw std::runtime_error(strerror(error));
      }
    }
  }

  bool BinaryClient::readCommand()
  {
    const size_t leftToRead = sizeof(Command) - myCommandRead;
    if (leftToRead > 0)
    {
      char * ptr = reinterpret_cast<char *>(&myCommand);
      const size_t data = readData(ptr + myCommandRead, leftToRead);
      myCommandRead += data;
    }
    return myCommandRead == sizeof(Command);
  }

  bool BinaryClient::readPayload()
  {
    myPayloadIn.resize(myCommand.length);

    const size_t leftToRead = myPayloadIn.size() - myPayloadRead;
    if (leftToRead > 0)
    {
      char * ptr = reinterpret_cast<char *>(myPayloadIn.data());
      const size_t data = readData(ptr + myPayloadRead, leftToRead);
      myPayloadRead += data;
    }
    return myPayloadRead == myPayloadIn.size();
  }

  void BinaryClient::sendReply(const BinaryBuffer & buffer, const uint8_t type, const uint32_t request, const uint8_t error)
  {
    const std::vector<uint8_t> & data = buffer.getData();

    Response response;

    response.stx = 0x02;
    response.version = 0x02;
    response.type = type;
    response.error = error;
    response.request = request;
    response.length = data.size();

    ssize_t sent1 = send(mySocket, &response, sizeof(Response), MSG_NOSIGNAL);
    throwIfError(sent1);
    if (sent1 == sizeof(Response) && !data.empty())
    {
      const ssize_t sent2 = send(mySocket, data.data(), data.size(), MSG_NOSIGNAL);
      throwIfError(sent2);
      sent1 += sent2;
    }

  #ifdef LOG_COMMANDS
    const bool ok = sent1 == (sizeof(Response) + data.size());
    const char * cmd = binarymonitor::getResponseStr(response.type);
    LogOutput("RESPONSE [%d]: CMD: 0x%02x, LEN: %7d, REQ: %8x, ERR: 0x%02x, OK: %d [%s]\n", mySocket, response.type, response.length, response.request, response.error, ok, cmd);
  #endif
  #ifdef LOG_PAYLOAD
    LogOutput("PAYLOAD:");
    for (size_t i = 0; i < std::min(30UL, data.size()); ++i)
    {
      LogOutput(" %02x", data[i]);
    }
    if (data.size() > 30UL)
    {
      LogOutput(" ...");
    }
    LogOutput("\n");
  #endif

  }

  void BinaryClient::sendBreakpointIfHit()
  {
    for (int i = 0; i < MAX_BREAKPOINTS; ++i)
    {
      const Breakpoint_t & bp = g_aBreakpoints[i];
      if (bp.bSet && bp.bHit)
      {
        LogOutput(">>> Hit: %d\n", i);
        sendBreakpoint(MON_EVENT_ID, i);
      }
    }
  }

  void BinaryClient::sendBreakpoint(const uint32_t request, const size_t i)
  {
    if (i >= MAX_BREAKPOINTS || !g_aBreakpoints[i].bSet)
    {
      throwBinaryException(e_MON_RESPONSE_CHECKPOINT_INFO, e_MON_ERR_INVALID_PARAMETER);
    }

    const Breakpoint_t & bp = g_aBreakpoints[i];

    const int operation = getBreakpointOperation(bp);

    BinaryBuffer buffer;
    buffer.writeInt32(i);                               // id
    buffer.writeInt8(bp.bHit);                          // hit
    buffer.writeInt16(bp.nAddress);                     // start
    buffer.writeInt16(bp.nAddress + bp.nLength - 1);    // end
    buffer.writeInt8(bp.bStop);                         // stop?
    buffer.writeInt8(bp.bEnabled);                      // enabled
    buffer.writeInt8(operation);                        // operation
    buffer.writeInt8(bp.bTemp);                         // temporary
    buffer.writeInt32(bp.nHitCount);                    // hit count
    buffer.writeInt32(0);                               // ignore count
    buffer.writeInt8(0);                                // condition?
    buffer.writeInt8(0);                                // memspace

    // logBreakpoint(bp);
    sendReply(buffer, e_MON_RESPONSE_CHECKPOINT_INFO, myCommand.request, e_MON_ERR_OK);
  }

  bool BinaryClient::process()
  {
    const bool newStopped = isStopped(g_nAppMode);
    if (newStopped != myStopped)
    {
      sendMonitorState(g_nAppMode);
      if (myStopped)
      {
        sendBreakpointIfHit();
      }
    }

    if (readCommand())
    {
      if (myCommand.stx != 0x02)
      {
        throw std::runtime_error("Invalid STX");
      }

      if (readPayload())
      {
        processCommand();
        reset();
        return true;
      }
    }
    return false;
  }

  void BinaryClient::cmdViceInfo()
  {
    BinaryBuffer buffer;

    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      std::vector<size_t> version;
      getVersion(version);
      for (size_t i : version)
      {
        buffer.writeInt8(static_cast<uint8_t>(i));
      }
    }

    {
      // no svn info
      BinaryBufferSize<uint8_t> binarySize(buffer);
      for (size_t i = 0; i < 4; ++i)
      {
        buffer.writeInt8(0);
      }
    }

    sendReply(buffer, e_MON_RESPONSE_VICE_INFO, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdResourceGet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_RESOURCE_GET);

    const std::string name = payload.readString();
    if (name == "MonitorServer")
    {
      sendResourceIntReply(myCommand.request, 0);
    }
    else if (name == "MonitorServerAddress")
    {
      sendResourceStringReply(myCommand.request, "ip4://127.0.0.1:6510");
    }
    else if (name == "VICIIPaletteFile")
    {
      sendResourceStringReply(myCommand.request, "pepto-pal");
    }
    else
    {
      throwBinaryException(e_MON_RESPONSE_RESOURCE_GET, e_MON_ERR_OBJECT_MISSING);
    }
  }

  void BinaryClient::cmdRegistersAvailable()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_REGISTERS_AVAILABLE);

    const uint8_t memspace = payload.read<uint8_t>();
    if (memspace != 0)
    {
      throwBinaryException(e_MON_RESPONSE_REGISTERS_AVAILABLE, e_MON_ERR_INVALID_MEMSPACE);
    }

    BinaryBuffer buffer;
    buffer.writeInt16(myAvailableRegisters.size());

    for (const auto & reg : myAvailableRegisters)
    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      buffer.writeInt8(reg.first);
      buffer.writeInt8(reg.second.size * 8);
      buffer.writeString(reg.second.name);
    }

    sendReply(buffer, e_MON_RESPONSE_REGISTERS_AVAILABLE, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdRegistersGet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_REGISTER_INFO);

    const uint8_t memspace = payload.read<uint8_t>();
    if (memspace != 0)
    {
      throwBinaryException(e_MON_RESPONSE_REGISTER_INFO, e_MON_ERR_INVALID_MEMSPACE);
    }

    sendRegisters(myCommand.request);
  }

  void BinaryClient::cmdRegistersSet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_REGISTER_INFO);

    const RegistersSet_t & registersSet = payload.read<RegistersSet_t>();

    if (registersSet.memspace != 0)
    {
      throwBinaryException(e_MON_RESPONSE_REGISTER_INFO, e_MON_ERR_INVALID_MEMSPACE);
    }

    for (size_t i = 0; i < registersSet.n; ++i)
    {
      PayloadBuffer registerPayload = payload.readSubBuffer();
      const uint8_t id = registerPayload.read<uint8_t>();
      const auto it = myAvailableRegisters.find(id);
      if (it == myAvailableRegisters.end())
      {
        throwBinaryException(e_MON_RESPONSE_REGISTER_INFO, e_MON_ERR_OBJECT_MISSING);
      }
      const Register_t & reg = it->second;
      const uint16_t value = registerPayload.read<uint16_t>();
      memcpy(reg.ptr, &value, reg.size);
    }

    sendRegisters(myCommand.request);
  }

  void BinaryClient::sendRegisters(const uint32_t request)
  {
    BinaryBuffer buffer;
    buffer.writeInt16(myAvailableRegisters.size());

    for (const auto & reg : myAvailableRegisters)
    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      buffer.writeInt8(reg.first);
      uint16_t value = 0;
      memcpy(&value, reg.second.ptr, reg.second.size);
      buffer.writeInt16(value);
    }

    sendReply(buffer, e_MON_RESPONSE_REGISTER_INFO, request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdDisplayGet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_DISPLAY_GET);
    payload.read<uint8_t>(); // discard

    const uint8_t format = payload.read<uint8_t>();
    if (format != 0) // RGBA
    {
      throwBinaryException(e_MON_RESPONSE_DISPLAY_GET, e_MON_ERR_INVALID_PARAMETER);
    }

    const uint32_t infoLength = 13;

    Video & video = GetVideo();
    const uint16_t width = video.GetFrameBufferWidth();
    const uint16_t height = video.GetFrameBufferHeight();
    const uint16_t x = video.GetFrameBufferBorderWidth();
    const uint16_t y = video.GetFrameBufferBorderHeight();
    const uint16_t sw = video.GetFrameBufferBorderlessWidth();
    const uint16_t sh = video.GetFrameBufferBorderlessHeight();

    typedef uint8_t pixel_t;

    const uint8_t bpp = sizeof(pixel_t) * 8;

    BinaryBuffer buffer;

    {
      BinaryBufferSize<uint32_t> binarySize(buffer);
      buffer.writeInt16(width);
      buffer.writeInt16(height);
      // I am not sure how this works, so we set the offset to 0.
      // buffer.writeInt16(x);
      // buffer.writeInt16(y);
      // buffer.writeInt16(sw);
      // buffer.writeInt16(sh);
      buffer.writeInt16(0);
      buffer.writeInt16(0);
      buffer.writeInt16(width);
      buffer.writeInt16(height);
      buffer.writeInt8(bpp);
    }

    const uint32_t displayBuffer = width * height * sizeof(pixel_t);
    buffer.writeInt32(displayBuffer);

    uint8_t * dest = buffer.enlargeBuffer(displayBuffer);
    const bgra_t * data = (const bgra_t *)video.GetFrameBuffer();

    for (size_t row = 0; row < height; ++row)
    {
      // the image is vertically flipped
      const bgra_t * sourceImage = data + (height - (row + 1)) * width;
      uint8_t * destImage = dest + row * width;

      for (size_t column = 0; column < width; ++column)
      {
        const bgra_t & source = sourceImage[column];
        const uint8_t grey = (source.b + source.g + source.r) / 3;
        destImage[column] = grey;
      }
    }

    sendReply(buffer, e_MON_RESPONSE_DISPLAY_GET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdPaletteGet()
  {
    BinaryBuffer buffer;
    const size_t n = 0x0100;
    buffer.writeInt16(n);
    for (size_t i = 0; i < n; ++i)
    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      buffer.writeInt8(i);
      buffer.writeInt8(i);
      buffer.writeInt8(i);
    }
    sendReply(buffer, e_MON_RESPONSE_PALETTE_GET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdBanksAvailable()
  {
    BinaryBuffer buffer;
    buffer.writeInt16(myBankNames.size());

    for (const auto & bank : myBankNames)
    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      buffer.writeInt16(bank.first);
      buffer.writeString(bank.second.c_str());
    }

    sendReply(buffer, e_MON_RESPONSE_BANKS_AVAILABLE, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::sendStopped()
  {
    if (!myStopped)
    {
      LogOutput(">>> Stopping... @ %04x\n", regs.pc);
      BinaryBuffer buffer;
      buffer.writeInt16(regs.pc);
      sendReply(buffer, e_MON_RESPONSE_STOPPED, MON_EVENT_ID, e_MON_ERR_OK);
      myStopped = true;
    }
  }

  void BinaryClient::cmdCheckpointSet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_CHECKPOINT_INFO);

    const CheckpointSet_t & checkpointSet = payload.read<CheckpointSet_t>();

    const int i = addBreakpoint(checkpointSet);

    // LogOutput("Add %d\n", int(i));
    // logBreakpoint(g_aBreakpoints[i]);

    sendBreakpoint(myCommand.request, i);
  }

  void BinaryClient::cmdCheckpointGet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_CHECKPOINT_INFO);

    const CheckpointGet_t & checkpointGet = payload.read<CheckpointGet_t>();

    sendBreakpoint(myCommand.request, checkpointGet.id);
  }

  void BinaryClient::cmdCheckpointToggle()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_CHECKPOINT_TOGGLE);

    const CheckpointToggle_t & checkpointToggle = payload.read<CheckpointToggle_t>();

    if (checkpointToggle.id >= MAX_BREAKPOINTS || !g_aBreakpoints[checkpointToggle.id].bSet)
    {
      throwBinaryException(e_MON_RESPONSE_CHECKPOINT_TOGGLE, e_MON_ERR_INVALID_PARAMETER);
    }

    Breakpoint_t & bp = g_aBreakpoints[checkpointToggle.id];
    bp.bEnabled = checkpointToggle.enabled;

    // LogOutput("Toggle %d\n", int(checkpointToggle.id));
    // logBreakpoint(bp);

    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_CHECKPOINT_TOGGLE, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdCheckpointDelete()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_CHECKPOINT_DELETE);

    const CheckpointDelete_t & checkpointDelete = payload.read<CheckpointDelete_t>();

    if (checkpointDelete.id >= MAX_BREAKPOINTS || !g_aBreakpoints[checkpointDelete.id].bSet)
    {
      throwBinaryException(e_MON_RESPONSE_CHECKPOINT_DELETE, e_MON_ERR_INVALID_PARAMETER);
    }

    // LogOutput("Delete %d\n", int(checkpointDelete.id));

    Breakpoint_t & bp = g_aBreakpoints[checkpointDelete.id];
    deleteBreakpoint(bp);

    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_CHECKPOINT_DELETE, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdCheckpointList()
  {
    size_t n = 0;
    for (int i = 0; i < MAX_BREAKPOINTS; ++i)
    {
      if (g_aBreakpoints[i].bSet)
      {
        ++n;
        sendBreakpoint(myCommand.request, i);
      }
    }

    BinaryBuffer buffer;
    buffer.writeInt32(n);
    sendReply(buffer, e_MON_RESPONSE_CHECKPOINT_LIST, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdMemoryGet()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_MEM_GET);

    const MemoryGet_t & memoryGet = payload.read<MemoryGet_t>();

    if (memoryGet.memspace != 0 || memoryGet.bankID != 0)
    {
      throwBinaryException(e_MON_RESPONSE_MEM_GET, e_MON_ERR_INVALID_PARAMETER);
    }

    BinaryBuffer buffer;
    const uint16_t length = memoryGet.endAddress - memoryGet.startAddress + 1;

    {
      BinaryBufferSize<uint16_t> binarySize(buffer);
      uint8_t * dest = buffer.enlargeBuffer(length);
      memcpy(dest, mem + memoryGet.startAddress, length);
    }

    sendReply(buffer, e_MON_RESPONSE_MEM_GET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdAutostart()
  {
    PayloadBuffer payload(myPayloadIn, e_MON_RESPONSE_AUTOSTART);

    const Autostart_t & autostart = payload.read<Autostart_t>();
    const std::string filename = payload.readString();

    CardManager & cardManager = GetCardMgr();
    const SS_CARDTYPE cardInSlot = cardManager.QuerySlot(6);
    Disk2InterfaceCard * card2 = dynamic_cast<Disk2InterfaceCard*>(cardManager.GetObj(6));
    const ImageError_e error = card2->InsertDisk(0, filename, IMAGE_USE_FILES_WRITE_PROTECT_STATUS, IMAGE_DONT_CREATE);

    if (error == eIMAGE_ERROR_NONE)
    {
      ResetMachineState();
      BinaryBuffer buffer;
      sendReply(buffer, e_MON_RESPONSE_AUTOSTART, myCommand.request, e_MON_ERR_OK);

      if (autostart.run)
      {
        enterMonitorState(MODE_RUNNING);
      }
    }
    else
    {
      throwBinaryException(e_MON_RESPONSE_AUTOSTART, e_MON_ERR_CMD_FAILURE);
    }
  }

  void BinaryClient::sendResourceStringReply(const uint32_t request, const char * value)
  {
    BinaryBuffer buffer;
    buffer.writeInt8(e_MON_RESOURCE_TYPE_STRING);
    buffer.writeString(value);
    sendReply(buffer, e_MON_RESPONSE_RESOURCE_GET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::sendResourceIntReply(const uint32_t request, const uint32_t value)
  {
    BinaryBuffer buffer;
    buffer.writeInt8(e_MON_RESOURCE_TYPE_INT);

    {
      BinaryBufferSize<uint8_t> binarySize(buffer);
      buffer.writeInt32(value);
    }

    sendReply(buffer, e_MON_RESPONSE_RESOURCE_GET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::sendError(const uint8_t type, const uint8_t error)
  {
    BinaryBuffer buffer;
    sendReply(buffer, type, myCommand.request, error);

    LogOutput("ERROR [%d]: CMD: 0x%02x, REQ: %8x, ERR: 0x%02x\n", mySocket, type, myCommand.request, error);
  }

  void BinaryClient::sendResume(const uint32_t request)
  {
    if (myStopped)
    {
      LogOutput(">>> Resuming...\n");
      BinaryBuffer buffer;
      buffer.writeInt16(regs.pc);
      sendReply(buffer, e_MON_RESPONSE_RESUMED, request, e_MON_ERR_OK);
      myStopped = false;
    }
  }

  void BinaryClient::cmdExit()
  {
    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_EXIT, myCommand.request, e_MON_ERR_OK);
    enterMonitorState(MODE_RUNNING);
  }

  void BinaryClient::cmdQuit()
  {
    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_QUIT, myCommand.request, e_MON_ERR_OK);

    removeAllBReakpoints();
    enterMonitorState(MODE_RUNNING);

    throw std::runtime_error("quit");
  }

  void BinaryClient::cmdPing()
  {
    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_PING, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::cmdReset()
  {
    BinaryBuffer buffer;
    sendReply(buffer, e_MON_RESPONSE_RESET, myCommand.request, e_MON_ERR_OK);
  }

  void BinaryClient::sendMonitorState(const AppMode_e mode)
  {
    switch (mode)
    {
    case MODE_RUNNING:
    case MODE_STEPPING:
      sendResume(MON_EVENT_ID);
      break;
    case MODE_DEBUG:
    case MODE_PAUSED:
      sendRegisters(MON_EVENT_ID);
      sendStopped();
      break;
    }
  }

  void BinaryClient::enterMonitorState(const AppMode_e mode)
  {
    if (myFrame->ChangeMode(mode))
    {
      sendMonitorState(mode);
    }
  }

  void BinaryClient::processCommand()
  {
  #ifdef LOG_COMMANDS
    const char * cmd = binarymonitor::getCommandStr(myCommand.type);
    LogOutput("COMMAND  [%d]: CMD: 0x%02x, LEN: %7d, REQ: %8x                   [%s]\n", mySocket, myCommand.type, myCommand.length, myCommand.request, cmd);
  #endif

    enterMonitorState(MODE_DEBUG);

    try
    {
      switch (myCommand.type)
      {
        case e_MON_CMD_VICE_INFO:
          cmdViceInfo();
          break;
        case e_MON_CMD_RESOURCE_GET:
          cmdResourceGet();
          break;
        case e_MON_CMD_REGISTERS_AVAILABLE:
          cmdRegistersAvailable();
          break;
        case e_MON_CMD_REGISTERS_GET:
          cmdRegistersGet();
          break;
        case e_MON_CMD_REGISTERS_SET:
          cmdRegistersSet();
          break;
        case e_MON_CMD_BANKS_AVAILABLE:
          cmdBanksAvailable();
          break;
        case e_MON_CMD_DISPLAY_GET:
          cmdDisplayGet();
          break;
        case e_MON_CMD_PALETTE_GET:
          cmdPaletteGet();
          break;
        case e_MON_CMD_CHECKPOINT_SET:
          cmdCheckpointSet();
          break;
        case e_MON_CMD_CHECKPOINT_GET:
          cmdCheckpointGet();
          break;
        case e_MON_CMD_CHECKPOINT_LIST:
          cmdCheckpointList();
          break;
        case e_MON_CMD_CHECKPOINT_DELETE:
          cmdCheckpointDelete();
          break;
        case e_MON_CMD_CHECKPOINT_TOGGLE:
          cmdCheckpointToggle();
          break;
        case e_MON_CMD_MEM_GET:
          cmdMemoryGet();
          break;
        case e_MON_CMD_PING:
          cmdPing();
          break;
        case e_MON_CMD_EXIT:
          cmdExit();
          break;
        case e_MON_CMD_AUTOSTART:
          cmdAutostart();
          break;
        case e_MON_CMD_RESET:
          cmdReset();
          break;
        case e_MON_CMD_QUIT:
          cmdQuit();
          break;
        default:
          throwBinaryException(myCommand.type, e_MON_ERR_CMD_INVALID_TYPE);
      }
    }
    catch (const BinaryException & e)
    {
      sendError(e.type, e.type);
    }

  #ifdef LOG_COMMANDS
    LogOutput("\n");
  #endif
  }

  std::shared_ptr<BinaryMonitor> BinaryMonitor::create(const std::string & address, common2::CommonFrame * frame)
  {
    sockaddr_in server;
    parseAddress(address, server);

    const int sk = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);

    if (bind(sk, (sockaddr *)&server , sizeof(server)) < 0)
    {
      LogOutput("Monitor: bind failed: %s\n", strerror(errno));
    }
    else if (listen(sk, 5))
    {
      LogOutput("Monitor: listen failed: %s\n", strerror(errno));
    }
    else
    {
      LogOutput("Monitor: listening for incoming connections\n");
      return std::make_shared<BinaryMonitor>(sk, frame);
    }

    close(sk);
    return nullptr;
  }

  BinaryMonitor::BinaryMonitor(const int socket, common2::CommonFrame * frame) : mySocket(socket), myFrame(frame)
  {
  }

  BinaryMonitor::~BinaryMonitor()
  {
    LogOutput("Monitor: closing socket\n");
    close(mySocket);
  }

  void BinaryMonitor::process()
  {
    sockaddr_in client;
    socklen_t len = sizeof(client);
    const int clientSocket = accept4(mySocket, (sockaddr *)&client, &len, 0);
    if (clientSocket >= 0)
    {
      myClients.push_back(std::make_shared<BinaryClient>(clientSocket, myFrame));
    }
    else
    {
      int error = errno;
      if (error != EAGAIN && error != EWOULDBLOCK)
      {
        LogOutput("Accept: %d\n", error);
      }
    }

    std::vector<std::shared_ptr<BinaryClient>>::iterator iter = myClients.begin();
    while (iter != myClients.end())
    {
      try
      {
        for (size_t i = 0; i < 4; ++i)
        {
          if (!(*iter)->process())
          {
            break;
          }
        }
        ++iter;
      }
      catch (const std::exception & e)
      {
        LogOutput("Client: %s\n", e.what());
        iter = myClients.erase(iter);
      }
    }
  }

}