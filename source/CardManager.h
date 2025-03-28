#pragma once

#include "Card.h"
#include "Disk2CardManager.h"
#include "LanguageCard.h"
#include "MockingboardCardManager.h"
#include "Common.h"

class CardManager
{
public:
	CardManager(void) :
		m_pMouseCard(NULL),
		m_pSSC(NULL),
		m_pParallelPrinterCard(NULL),
		m_pZ80Card(NULL)
	{
		InsertInternal(SLOT0, CT_Empty);
		InsertInternal(SLOT1, CT_GenericPrinter);
		InsertInternal(SLOT2, CT_SSC);
		InsertInternal(SLOT3, CT_Empty);
		InsertInternal(SLOT4, CT_Empty);
		InsertInternal(SLOT5, CT_Empty);
		InsertInternal(SLOT6, CT_Disk2);
		InsertInternal(SLOT7, CT_Empty);
		InsertAuxInternal(CT_Extended80Col);	// For Apple //e and above
	}
	~CardManager(void)
	{
		for (UINT i=0; i<NUM_SLOTS; i++)
			RemoveInternal(i);
		RemoveAuxInternal();
	}

	void Insert(UINT slot, SS_CARDTYPE type, bool updateRegistry = true);
	void Remove(UINT slot, bool updateRegistry = true);
	SS_CARDTYPE QuerySlot(UINT slot) { _ASSERT(slot<NUM_SLOTS); return m_slot[slot]->QueryType(); }
	Card& GetRef(UINT slot)
	{
		_ASSERT(m_slot[slot]);
		return *m_slot[slot];
	}
	Card* GetObj(UINT slot)
	{
		return m_slot[slot];
	}

	void InsertAux(SS_CARDTYPE type, bool updateRegistry = true);
	void RemoveAux(void);
	SS_CARDTYPE QueryAux(void) { return m_aux->QueryType(); }
	Card* GetObjAux(void) { _ASSERT(0); return m_aux; }	// ASSERT because this is a DummyCard

	//

	Disk2CardManager& GetDisk2CardMgr(void) { return m_disk2CardMgr; }
	LanguageCardManager& GetLanguageCardMgr(void) { return m_languageCardMgr; }
	MockingboardCardManager& GetMockingboardCardMgr(void) { return m_mockingboardCardMgr; }
	class CMouseInterface* GetMouseCard(void) { return m_pMouseCard; }
	bool IsMouseCardInstalled(void) { return m_pMouseCard != NULL; }
	class CSuperSerialCard* GetSSC(void) { return m_pSSC; }
	bool IsSSCInstalled(void) { return m_pSSC != NULL; }
	class ParallelPrinterCard* GetParallelPrinterCard(void) { return m_pParallelPrinterCard; }
	bool IsParallelPrinterCardInstalled(void) { return m_pParallelPrinterCard != NULL; }

	void InitializeIO(LPBYTE pCxRomPeripheral);
	void Destroy(void);
	void Reset(const bool powerCycle);
	void Update(const ULONG nExecutedCycles);
	void SaveSnapshot(YamlSaveHelper& yamlSaveHelper);

private:
	void InsertInternal(UINT slot, SS_CARDTYPE type);
	void InsertAuxInternal(SS_CARDTYPE type);
	void RemoveInternal(UINT slot);
	void RemoveAuxInternal(void);

	Card* m_slot[NUM_SLOTS];
	Card* m_aux;
	Disk2CardManager m_disk2CardMgr;
	LanguageCardManager m_languageCardMgr;
	MockingboardCardManager m_mockingboardCardMgr;
	class CMouseInterface* m_pMouseCard;
	class CSuperSerialCard* m_pSSC;
	class ParallelPrinterCard* m_pParallelPrinterCard;
	class Z80Card* m_pZ80Card;
};
