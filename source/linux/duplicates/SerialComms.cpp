#include "StdAfx.h"

#include "Common.h"
#include "SerialComms.h"
#include "YamlHelper.h"


CSuperSerialCard::CSuperSerialCard(UINT slot) :
  Card(CT_SSC), m_uSlot(slot)
{
}

CSuperSerialCard::~CSuperSerialCard()
{
}

void CSuperSerialCard::SetSerialPortName(const char* pSerialPortName)
{
}

void CSuperSerialCard::CommReset()
{
}

void CSuperSerialCard::CommInitialize(LPBYTE pCxRomPeripheral, UINT uSlot)
{
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

void CSuperSerialCard::LoadSnapshotDIPSW(YamlLoadHelper& yamlLoadHelper, std::string key, SSC_DIPSW& dipsw)
{
  if (!yamlLoadHelper.GetSubMap(key))
    throw std::string("Card: Expected key: " + key);

  yamlLoadHelper.LoadUint(SS_YAML_KEY_BAUDRATE);
  yamlLoadHelper.LoadUint(SS_YAML_KEY_FWMODE);
  yamlLoadHelper.LoadUint(SS_YAML_KEY_STOPBITS);
  yamlLoadHelper.LoadUint(SS_YAML_KEY_BYTESIZE);
  yamlLoadHelper.LoadUint(SS_YAML_KEY_PARITY);
  yamlLoadHelper.LoadBool(SS_YAML_KEY_LINEFEED);
  yamlLoadHelper.LoadBool(SS_YAML_KEY_INTERRUPTS);

  yamlLoadHelper.PopMap();
}

bool CSuperSerialCard::LoadSnapshot(YamlLoadHelper& yamlLoadHelper, UINT slot, UINT version)
{
  if (slot != 2)	// fixme
    throw std::string("Card: wrong slot");

  if (version < 1 || version > kUNIT_VERSION)
    throw std::string("Card: wrong version");

  SSC_DIPSW dipsw;
  LoadSnapshotDIPSW(yamlLoadHelper, SS_YAML_KEY_DIPSWDEFAULT, dipsw);
  LoadSnapshotDIPSW(yamlLoadHelper, SS_YAML_KEY_DIPSWCURRENT, dipsw);

  if (version == 1)	// Consume redundant/obsolete data
  {
    yamlLoadHelper.LoadUint(SS_YAML_KEY_PARITY);		// Redundant: derived from uCommandByte in UpdateCommandReg()
    yamlLoadHelper.LoadBool(SS_YAML_KEY_TXIRQENABLED);	// Redundant: derived from uCommandByte in UpdateCommandReg()
    yamlLoadHelper.LoadBool(SS_YAML_KEY_RXIRQENABLED);	// Redundant: derived from uCommandByte in UpdateCommandReg()

    yamlLoadHelper.LoadUint(SS_YAML_KEY_BAUDRATE);		// Redundant: derived from uControlByte in UpdateControlReg()
    yamlLoadHelper.LoadUint(SS_YAML_KEY_STOPBITS);		// Redundant: derived from uControlByte in UpdateControlReg()
    yamlLoadHelper.LoadUint(SS_YAML_KEY_BYTESIZE);		// Redundant: derived from uControlByte in UpdateControlReg()

    yamlLoadHelper.LoadUint(SS_YAML_KEY_INACTIVITY);	// Obsolete (so just consume)
  }
  else if (version >= 2)
  {
    yamlLoadHelper.LoadBool(SS_YAML_KEY_SUPPORT_DCD);
  }

  yamlLoadHelper.LoadUint(SS_YAML_KEY_COMMAND);
  yamlLoadHelper.LoadUint(SS_YAML_KEY_CONTROL);

  yamlLoadHelper.LoadBool(SS_YAML_KEY_TXIRQPENDING);
  yamlLoadHelper.LoadBool(SS_YAML_KEY_RXIRQPENDING);
  yamlLoadHelper.LoadBool(SS_YAML_KEY_WRITTENTX);

  yamlLoadHelper.LoadString(SS_YAML_KEY_SERIALPORTNAME);

  return true;
}

void CSuperSerialCard::SaveSnapshot(YamlSaveHelper&)
{
}

std::string CSuperSerialCard::GetSnapshotCardName()
{
  return "Super Serial Card";
}
