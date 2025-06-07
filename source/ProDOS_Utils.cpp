#include "StdAfx.h"

#include "ProDOS_Utils.h"
#include "ProDOS_FileSystem.h"
#include "Core.h"
#include "DiskImage.h"
#include "DiskImageHelper.h"
#include "FrameBase.h"
#include "../resource/resource.h"

//===========================================================================
inline int Util_GetTrackSectorOffset( const int nTrack, const int nSector )
{
	return (TRACK_DENIBBLIZED_SIZE * nTrack) + (nSector * 256);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

enum { DSK_SECTOR_SIZE = 256 };

// Map Physical <-> Logical
const uint8_t g_aInterleave_DSK[ 16 ] =
{
  //0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F // logical order
  0x0,0xE,0xD,0xC,0xB,0xA,0x9,0x8,0x7,0x6,0x5,0x4,0x3,0x2,0x1,0xF // physical order
};

void Util_Disk_InterleaveForward( int iSector, size_t *src_, size_t *dst_ )
{
  *src_ = g_aInterleave_DSK[ iSector ]*DSK_SECTOR_SIZE;
  *dst_ =                    iSector  *DSK_SECTOR_SIZE; // linearize
};

void Util_Disk_InterleaveReverse( int iSector, size_t *src_, size_t *dst_ )
{
  *src_ =                    iSector  *DSK_SECTOR_SIZE; // un-linearize
  *dst_ = g_aInterleave_DSK[ iSector ]*DSK_SECTOR_SIZE;
}

// With C++17 could use std::filesystem::path
// returns '.dsk'
// Alt. PathFindExtensionA()
std::string Util_GetFileNameExtension(const std::string& pathname)
{
#if _MSVC_LANG >= 201703L // Compiler option: /std:c++17
  const std::filesystem::path path( pathname );
  return path.extension().generic_string();
#else
  const size_t nLength    = pathname.length();
  const size_t iExtension = pathname.rfind( '.', nLength );
  if (iExtension != std::string::npos)
    return pathname.substr( iExtension, nLength );
  else
    return std::string("");
#endif
}

SectorOrder_e Util_Disk_CalculateSectorOrder(const std::string& pathname)
{
  SectorOrder_e eOrder = INTERLEAVE_AUTO_DETECT;
  std::string sExtension = Util_GetFileNameExtension( pathname );
  if (sExtension.length())
  {
    /**/ if (sExtension == ".dsk") eOrder = INTERLEAVE_DOS33_ORDER;
    else if (sExtension == ".do" ) eOrder = INTERLEAVE_DOS33_ORDER;
    else if (sExtension == ".po" ) eOrder = INTERLEAVE_PRODOS_ORDER;
    else if (sExtension == ".hdv") eOrder = INTERLEAVE_PRODOS_ORDER;
  }
  return eOrder;
}

// Swizzle sectors in DOS33 order to ProDOS order in-place
//===========================================================================
void Util_ProDOS_ForwardSectorInterleave (uint8_t *pDiskBytes, const size_t nDiskSize, const SectorOrder_e eSectorOrder)
{
	if (eSectorOrder == INTERLEAVE_DOS33_ORDER)
	{
		size_t   nOffset = 0;
		uint8_t *pSource = new uint8_t[ nDiskSize ];
		memcpy( pSource, pDiskBytes, nDiskSize );

		const int nTracks = nDiskSize / TRACK_DENIBBLIZED_SIZE;
		for( int iTrack = 0; iTrack < nTracks; iTrack++ )
		{
			for( int iSector = 0; iSector < 16; iSector++ )
			{
				size_t nSrc;
				size_t nDst;
				Util_Disk_InterleaveForward( iSector, &nSrc, &nDst );
				memcpy( pDiskBytes + nOffset + nDst, pSource + nOffset + nSrc, DSK_SECTOR_SIZE );
			}
			nOffset += TRACK_DENIBBLIZED_SIZE;
		}

		delete [] pSource;
	}
}

// See:
// * https://prodos8.com/docs/technote/30/
// * 2.2.3 Sparse Files
//   https://prodos8.com/docs/techref/file-use/
// * B.3.6 - Sparse Files
//   https://prodos8.com/docs/techref/file-organization/
//===========================================================================
bool Util_ProDOS_IsFileBlockSparse( int nOffset, const uint8_t* pFileData, const size_t nFileSize )
{
	if ((size_t)nOffset >= nFileSize)
		return false;

	const int nEnd   = (nOffset + PRODOS_BLOCK_SIZE ) > nFileSize
		? nOffset + (nFileSize % PRODOS_BLOCK_SIZE)
		: nOffset +              PRODOS_BLOCK_SIZE;
	for (int iOffset = nOffset ; iOffset < nEnd; iOffset++ )
	{
		if (pFileData[ iOffset ])
		{
			return false;
		}
	}
	return true;
}

// Swizzles sectors in ProDOS order to DOS33 order in-place
//===========================================================================
void Util_ProDOS_ReverseSectorInterleave (uint8_t *pDiskBytes, const size_t nDiskSize, const SectorOrder_e eSectorOrder)
{
	// Swizle from ProDOS to DOS33 order
	if (eSectorOrder == INTERLEAVE_DOS33_ORDER)
	{
		size_t   nOffset = 0;
		uint8_t *pSource = new uint8_t[ nDiskSize ];
		memcpy( pSource, pDiskBytes, nDiskSize );

		const int nTracks = nDiskSize / TRACK_DENIBBLIZED_SIZE;
		for( int iTrack = 0; iTrack < nTracks; iTrack++ )
		{
			for( int iSector = 0; iSector < 16; iSector++ )
			{
				size_t nSrc;
				size_t nDst;
				Util_Disk_InterleaveReverse( iSector, &nSrc, &nDst );
				memcpy( pDiskBytes + nOffset + nDst, pSource + nOffset + nSrc, DSK_SECTOR_SIZE );
			}
			nOffset += TRACK_DENIBBLIZED_SIZE;
		}

		delete [] pSource;
	}
}

// NB Assumes pDiskBytes are in ProDOS order!
//===========================================================================
void Util_ProDOS_FormatFileSystem (uint8_t *pDiskBytes, const size_t nDiskSize, const char *pVolumeName)
{
	const size_t nBootBlockSize = DSK_SECTOR_SIZE * 2;
	memset( pDiskBytes + nBootBlockSize, 0, nDiskSize - nBootBlockSize );

	// Create blocks for root directory
	int nRootDirBlocks = 4;
	int iPrevDirBlock  = 0;
	int iNextDirBlock  = 0;
	int iOffset;

	ProDOS_VolumeHeader_t  tVolume;
	ProDOS_VolumeHeader_t *pVolume = &tVolume;
	memset( pVolume, 0, sizeof(ProDOS_VolumeHeader_t) );

	// Init Bitmap
	pVolume->meta.bitmap_block = (uint16_t) (PRODOS_ROOT_BLOCK + nRootDirBlocks);
	int nBitmapBlocks = ProDOS_BlockInitFree( pDiskBytes, nDiskSize, pVolume );

	// Set boot blocks as in-use
	for( int iBlock = 0; iBlock < PRODOS_ROOT_BLOCK; iBlock++ )
		ProDOS_BlockSetUsed( pDiskBytes, pVolume, iBlock );

	for( int iBlock = 0; iBlock < nRootDirBlocks; iBlock++ )
	{
		iNextDirBlock = ProDOS_BlockGetFirstFree( pDiskBytes, nDiskSize, pVolume );
		iOffset       = iNextDirBlock * PRODOS_BLOCK_SIZE;
		ProDOS_BlockSetUsed( pDiskBytes, pVolume, iNextDirBlock );

		// Double Linked List
		// [0] = prev
		// [2] = next -- will be set on next allocation
		ProDOS_Put16( pDiskBytes, iOffset + 0, iPrevDirBlock );
		ProDOS_Put16( pDiskBytes, iOffset + 2, 0 );

		if( iBlock )
		{
			// Fixup previous directory block with pointer to this one
			iOffset = iPrevDirBlock * PRODOS_BLOCK_SIZE;
			ProDOS_Put16( pDiskBytes, iOffset + 2, iNextDirBlock );
		}

		iPrevDirBlock = iNextDirBlock;
	}

	// Alloc Bitmap Blocks
	for( int iBlock = 0; iBlock < nBitmapBlocks; iBlock++ )
	{
		int iBitmap = ProDOS_BlockGetFirstFree( pDiskBytes, nDiskSize, pVolume );
		ProDOS_BlockSetUsed( pDiskBytes, pVolume, iBitmap );
	}

	pVolume->entry_len  = 0x27;
	pVolume->entry_num  = (uint8_t) (PRODOS_BLOCK_SIZE / pVolume->entry_len);
	pVolume->file_count = 0;

	if( *pVolumeName == '/' )
		pVolumeName++;

	size_t nLen = strlen( pVolumeName );

	pVolume->kind = PRODOS_KIND_ROOT;
	pVolume->len  = (uint8_t) nLen;
	ProDOS_String_CopyUpper( pVolume->name, pVolumeName, 15 );

	pVolume->access = 0
		| ACCESS_D
		| ACCESS_N
		| ACCESS_B
		| ACCESS_W
		| ACCESS_R
		;

	// TODO:
	//tVolume.date   = config.date;
	//tVolume.time   = config.time;

	ProDOS_SetVolumeHeader( pDiskBytes, pVolume, PRODOS_ROOT_BLOCK );
}

//===========================================================================
bool Util_ProDOS_AddFile (uint8_t* pDiskBytes, const size_t nDiskSize, const char* pVolumeName, const uint8_t* pFileData, const size_t nFileSize, ProDOS_FileHeader_t &meta, const bool bAllowSparseFile = false)
{
	assert( pFileData );

	int iBase = ProDOS_BlockGetPathOffset( pDiskBytes, nullptr, "/" ); // On an empty disk this will be PRODOS_ROOT_OFFSET
	int iDirBlock = iBase / PRODOS_BLOCK_SIZE;

	ProDOS_VolumeHeader_t  tVolume;
	ProDOS_VolumeHeader_t *pVolume = &tVolume;
	memset( pVolume, 0, sizeof(ProDOS_VolumeHeader_t) );

	ProDOS_GetVolumeHeader( pDiskBytes, pVolume, iDirBlock );

	// Verify we have room in the current directory
	int iFreeOffset   = ProDOS_DirGetFirstFreeEntryOffset( pDiskBytes, pVolume, iBase );
	if (iFreeOffset <= 0)
		return false; // disk full

	int iKind        = PRODOS_KIND_DEL;
	int nBlocksData  = (nFileSize + PRODOS_BLOCK_SIZE - 1) / PRODOS_BLOCK_SIZE;
	int nBlocksIndex = 0; // Num Blocks needed for meta (Index)
	int nBlocksTotal = 0;
	int nBlocksFree  = 0;
	int iNode        = 0; // Master Index, Single Index, or 0 if none
	int iIndexBase   = 0; // Single Index
	int iMasterIndex = 0; // master block points to N IndexBlocks

	// Calculate size of meta blocks and kind of file
	if (nFileSize <=   1*PRODOS_BLOCK_SIZE) // <= 512, 1 Block
	{
		iKind = PRODOS_KIND_SEED;
	}
	else
	if (nFileSize > 256*PRODOS_BLOCK_SIZE) // >= 128K, 257-65536 Blocks
	{
		iKind        = PRODOS_KIND_TREE;
		nBlocksIndex = (nBlocksData + (PRODOS_BLOCK_SIZE/2-1)) / (PRODOS_BLOCK_SIZE / 2);
		nBlocksIndex++; // include master index block
	}
	else
	if( nFileSize > PRODOS_BLOCK_SIZE ) // <= 128K, 2-256 blocks
	{
		iKind        = PRODOS_KIND_SAPL;
		nBlocksIndex = 1; // single index = PRODOS_BLOCK_SIZE/2 = 256 data blocks
	}

	// We simply can't set nBlocksTotal
	//     nBlocksTotal = nBlocksIndex + nBlocksData;
	// Since we may have sparse blocks
	nBlocksTotal = nBlocksIndex;

	for( int iBlock = 0; iBlock < nBlocksIndex; iBlock++ )
	{
		// Blank Volume
		//  0 ] Boot Apple //e
		//  1 ] Boot Apple ///
		//  2 \ Root Directory
		//  3 |
		//  4 |
		//  5 /
		//  6   Volume Bitmap
		//  7   First Free Block
		int iMetaBlock = ProDOS_BlockGetFirstFree( pDiskBytes, nDiskSize, pVolume );
		if( !iMetaBlock )
		{
			return false; // out of disk space, no free room
		}

		// First Block has meta information -- blocks used for file
		if (iBlock == 0)
		{
			iNode      = iMetaBlock;
			iIndexBase = iMetaBlock * PRODOS_BLOCK_SIZE;
		}
		else
		{
			// PRODOS_KIND_SEED doesn't have an index block
			if (iKind == PRODOS_KIND_TREE)
			{
				ProDOS_PutIndexBlock( pDiskBytes, iMasterIndex, iBlock, iMetaBlock );
			}
			// Not implemented PRODOS_KIND_TREE
			assert( iKind != PRODOS_KIND_TREE );
		}

		ProDOS_BlockSetUsed( pDiskBytes, pVolume, iMetaBlock );
#if _DEBUG
	LogOutput( "0x----  FileBlock: --/--  MetaBlock: $%02X\n", iMetaBlock );
#endif
	}

	// Copy Data
	const uint8_t *pSrc           = pFileData;
	const size_t   nSlack         = nFileSize % PRODOS_BLOCK_SIZE;
	const size_t   nLastBlockSize = nSlack ? nSlack : PRODOS_BLOCK_SIZE;
	const bool PRODOS_FILE_TYPE_TREE_NOT_IMPLEMENTED = false;

	for( int iBlock = 0; iBlock < nBlocksData; iBlock++ )
	{
		int  iDataBlock = ProDOS_BlockGetFirstFree( pDiskBytes, nDiskSize, pVolume );
		if( !iDataBlock )
		{
			return false;
		}

		// if file size <= 512 bytes
		if (iBlock == 0)
		{
			if (iKind == PRODOS_KIND_SEED)
			{
				iNode = iDataBlock;
			}
		}

		int      iDstOffset = iDataBlock * PRODOS_BLOCK_SIZE;
		uint8_t *pDst       = pDiskBytes + iDstOffset;
		bool     bLastBlock = iBlock == (nBlocksData - 1);

		// Any 512-byte block containing all zeroes doesn't need to be written to disk.
		// Instead we write an index block of $0000 to tell ProDOS that this block is a sparse block.
		bool bIsSparseBlock = Util_ProDOS_IsFileBlockSparse( iBlock * PRODOS_BLOCK_SIZE, pFileData, nFileSize );

#if _DEBUG
	LogOutput( "0x%04X  FileBlock: %2d/%2d  DataBlock: $%02X  LastBlock=%d  Sparse=%d\n"
		, iBlock*PRODOS_BLOCK_SIZE
		, iBlock+1
		, nBlocksData
		, iDataBlock
		, bLastBlock
		, bIsSparseBlock
	);
#endif
		if (bIsSparseBlock && bAllowSparseFile)
		{
			if (iKind == PRODOS_KIND_SAPL)
			{
				ProDOS_PutIndexBlock( pDiskBytes, iIndexBase, iBlock, 0 ); // Update File Bitmap
			}
			else
			if (iKind == PRODOS_KIND_TREE)
			{
				assert( PRODOS_FILE_TYPE_TREE_NOT_IMPLEMENTED ); // TODO: ProDOS tree file not implemented
			}
		}
		else
		{
			if (bLastBlock)
			{
				if (nLastBlockSize)
				{
					memcpy( pDst, pSrc, nLastBlockSize );
					nBlocksTotal++;
					ProDOS_BlockSetUsed( pDiskBytes, pVolume, iDataBlock ); // Update Volume Bitmap
				}
				else
				{
					iIndexBase = 0; // Last block has zero size. Don't update index block
				}
			}
			else
			{
				memcpy( pDst, pSrc, PRODOS_BLOCK_SIZE );
				nBlocksTotal++;
				ProDOS_BlockSetUsed( pDiskBytes, pVolume, iDataBlock ); // Update Volume Bitmap
			}

			if (iKind == PRODOS_KIND_SAPL)
			{
				// Update single index block
				if (iIndexBase)
				{
					ProDOS_PutIndexBlock( pDiskBytes, iIndexBase, iBlock, iDataBlock ); // Update File Bitmap
				}
			}
			else
			if (iKind == PRODOS_KIND_TREE)
			{
				// Update multiple index blocks
				assert( PRODOS_FILE_TYPE_TREE_NOT_IMPLEMENTED ); // TODO: ProDOS tree file not implemented
			}
		}
		pSrc += PRODOS_BLOCK_SIZE;
	}

	// Update directory entry with meta information
	meta.inode     = iNode;
	meta.blocks    = nBlocksTotal; // Note: File Entry DOES include index + non-sparse data block(s)
	meta.dir_block = iDirBlock;
	ProDOS_PutFileHeader( pDiskBytes, iFreeOffset, &meta );

	// Update Directory with Header
	pVolume->file_count++;
	ProDOS_SetVolumeHeader( pDiskBytes, pVolume, iDirBlock );

	return true;
}

//===========================================================================
bool Util_ProDOS_CopyBASIC ( uint8_t *pDiskBytes, const size_t nDiskSize, const char *pVolumeName, FrameBase *pWinFrame )
{
	const size_t   nFileSize = 0x2800; // 10,240 -> 20 blocks + 1 index block = 21 blocks
	const uint8_t *pFileData = (uint8_t *) pWinFrame->GetResource(IDR_FILE_BASIC17, "FIRMWARE", nFileSize);

	// Acc dnb??iwr /PRODOS.2.4.3    Blocks Size    Type    Aux   Kind  iNode Dir   Ver Min  Create    Time    Modified  Time
	// --- -------- ---------------- ------ ------- ------- ----- ----- ----- ----- --- ---  --------- ------  --------- ------
	// $21 --b----r *BASIC.SYSTEM        21 $002800 SYS $FF $2000 sap 2 @0029 @0002 2.4 v00  13-JAN-18  9:09a  30-AUG-16  7:56a
	int bAccess = 0
		| ACCESS_B
		| ACCESS_R
		;
	ProDOS_FileHeader_t meta;
	memset( &meta, 0, sizeof(ProDOS_FileHeader_t) );

	const char    *pName = "BASIC.SYSTEM";
	const uint16_t nDateCreate = ProDOS_PackDate( 18, 1, 13 ); // YYMMDD, NOTE: Jan starts at 1, not 0.
	const uint16_t nTimeCreate = ProDOS_PackTime( 9, 9 );
	const uint16_t nDateModify = ProDOS_PackDate( 16, 8, 30 ); // YYMMDD
	const uint16_t nTimeModify = ProDOS_PackTime( 7, 56 );

	meta.kind      = PRODOS_KIND_SAPL;
	strcpy( meta.name, pName );
	meta.len       = strlen( pName ) & 0xF;
	meta.type      = 0xFF; // SYS
	//  .inode     = TBD
	//  .blocks    = TBD
	meta.size      = nFileSize;
	meta.date      = nDateCreate;
	meta.time      = nTimeCreate;
	meta.cur_ver   = 0x24;
	meta.min_ver   = 0x00;
	meta.access    = bAccess;
	meta.aux       = 0x2000;
	meta.mod_date  = nDateModify;
	meta.mod_time  = nTimeModify;
	//  .dir_block = TBD;

	return Util_ProDOS_AddFile( pDiskBytes, nDiskSize, pVolumeName, pFileData, nFileSize, meta, true );
}

//=======Util_ProDOS_CopyBitsyBoot====================================================================
bool Util_ProDOS_CopyBitsyBoot( uint8_t *pDiskBytes, const size_t nDiskSize, const char *pVolumeName, FrameBase *pWinFrame )
{
	const size_t   nFileSize = 0x16D; // < 512 bytes -> SEED, only 1 data block
	const uint8_t *pFileData = (uint8_t *) pWinFrame->GetResource(IDR_FILE_BITSY_BOOT, "FIRMWARE", nFileSize);

	// Acc dnb??iwr /PRODOS.2.4.2    Blocks Size    Type    Aux   Kind  iNode Dir   Ver Min  Create    Time    Modified  Time
	// --- -------- ---------------- ------ ------- ------- ----- ----- ----- ----- --- ---  --------- ------  --------- ------
	// $21 --b----r *BITSY.BOOT           1 $00016D SYS $FF $2000 sed 1 @003D @0002 2.4 v00  13-JAN-18  9:09a  15-SEP-16  9:49a
	int bAccess = 0
		| ACCESS_B
		| ACCESS_R
		;

	ProDOS_FileHeader_t meta;
	memset( &meta, 0, sizeof(ProDOS_FileHeader_t) );

	const char    *pName = "BITSY.BOOT";
	const uint16_t nDateCreate = ProDOS_PackDate( 18, 1, 13 ); // YYMMDD, NOTE: Jan starts at 1, not 0.
	const uint16_t nTimeCreate = ProDOS_PackTime( 9, 9 );
	const uint16_t nDateModify = ProDOS_PackDate( 16, 9, 15 ); // YYMMDD
	const uint16_t nTimeModify = ProDOS_PackTime( 9, 49 );

	meta.kind      = PRODOS_KIND_SEED; // File size is <= 512 bytes
	strcpy( meta.name, pName );
	meta.len       = strlen( pName ) & 0xF;
	meta.type      = 0xFF; // SYS
	//  .inode     = TBD
	//  .blocks    = TBD
	meta.size      = nFileSize;
	meta.date      = nDateCreate;
	meta.time      = nTimeCreate;
	meta.cur_ver   = 0x24;
	meta.min_ver   = 0x00;
	meta.access    = bAccess;
	meta.aux       = 0x2000;
	meta.mod_date  = nDateModify;
	meta.mod_time  = nTimeModify;
	//  .dir_block = TBD;

	return Util_ProDOS_AddFile( pDiskBytes, nDiskSize, pVolumeName, pFileData, nFileSize, meta );
}

//===========================================================================
bool Util_ProDOS_CopyBitsyBye ( uint8_t *pDiskBytes, const size_t nDiskSize, const char *pVolumeName, FrameBase *pWinFrame )
{
	const size_t   nFileSize = 0x0038; // < 512 bytes -> SEED, only 1 data block
	const uint8_t *pFileData = (uint8_t*) pWinFrame->GetResource(IDR_FILE_BITSY_BYE, "FIRMWARE", nFileSize);

	// Acc dnb??iwr /PRODOS.2.4.2    Blocks Size    Type    Aux   Kind  iNode Dir   Ver Min  Create    Time    Modified  Time
	// --- -------- ---------------- ------ ------- ------- ----- ----- ----- ----- --- ---  --------- ------  --------- ------
	// $21 --b----r *QUIT.SYSTEM          1 $000038 SYS $FF $2000 sed 1 @0027 @0002 2.4 v00  13-JAN-18  9:09a  15-SEP-16  9:41a
	int bAccess = 0
		| ACCESS_B
		| ACCESS_R
		;

	ProDOS_FileHeader_t meta;
	memset( &meta, 0, sizeof(ProDOS_FileHeader_t) );

	const char    *pName = "QUIT.SYSTEM";
	const uint16_t nDateCreate = ProDOS_PackDate( 18, 1, 13 ); // YYMMDD, NOTE: Jan starts at 1, not 0.
	const uint16_t nTimeCreate = ProDOS_PackTime( 9, 9 );
	const uint16_t nDateModify = ProDOS_PackDate( 16, 9, 15 ); // YYMMDD
	const uint16_t nTimeModify = ProDOS_PackTime( 9, 41 );

	meta.kind      = PRODOS_KIND_SEED; // File size is <= 512 bytes
	strcpy( meta.name, pName );
	meta.len       = strlen( pName ) & 0xF;
	meta.type      = 0xFF; // SYS
	//  .inode     = TBD
	//  .blocks    = TBD
	meta.size      = nFileSize;
	meta.date      = nDateCreate;
	meta.time      = nTimeCreate;
	meta.cur_ver   = 0x24;
	meta.min_ver   = 0x00;
	meta.access    = bAccess;
	meta.aux       = 0x2000;
	meta.mod_date  = nDateModify;
	meta.mod_time  = nTimeModify;
	//  .dir_block = TBD;

	return Util_ProDOS_AddFile( pDiskBytes, nDiskSize, pVolumeName, pFileData, nFileSize, meta );
}

//===========================================================================
bool Util_ProDOS_CopyDOS ( uint8_t *pDiskBytes, const size_t nDiskSize, const char *pVolumeName, FrameBase *pWinFrame )
{
	const size_t   nFileSize = 0x42E8; // 17,128 -> 34 blocks but last block is sparse -> 33 data blocks + 1 index block
	const uint8_t *pFileData = (uint8_t*) pWinFrame->GetResource(IDR_OS_PRODOS243, "FIRMWARE", nFileSize);

	// Acc dnb??iwr /PRODOS.2.4.3    Blocks Size    Type    Aux   Kind  iNode Dir   Ver Min  Create    Time    Modified  Time
	// --- -------- ---------------- ------ ------- ------- ----- ----- ----- ----- --- ---  --------- ------  --------- ------
	// $E3 dnb---wr  PRODOS              34 $0042E8 SYS $FF $0000 sap 2 @0007 @0002 0.0 v80  30-DEC-23  2:43a  30-DEC-23  2:43a
	int bAccess = 0
		| ACCESS_D
		| ACCESS_N
		| ACCESS_B
		| ACCESS_W
		| ACCESS_R
		;
	ProDOS_FileHeader_t meta;
	memset( &meta, 0, sizeof(ProDOS_FileHeader_t) );

	const char *pName = "PRODOS";
	const uint16_t nDateCreate = ProDOS_PackDate( 23, 12, 30 ); // NOTE: Jan starts at 1, not 0.
	const uint16_t nTimeCreate = ProDOS_PackTime( 2, 43 ); // Version.  "Cute".
	const uint16_t nDateModify = ProDOS_PackDate( 23, 12, 30 );
	const uint16_t nTimeModify = ProDOS_PackTime( 2, 43 );

	meta.kind      = PRODOS_KIND_SAPL;
	strcpy( meta.name, pName );
	meta.len       = strlen( pName ) & 0xF;
	meta.type      = 0xFF; // SYS
	//  .inode     = TBD
	//  .blocks    = TBD
	meta.size      = nFileSize;
	meta.date      = nDateCreate;
	meta.time      = nTimeCreate;
	meta.cur_ver   = 0x00;
	meta.min_ver   = 0x80;
	meta.access    = bAccess;
	meta.aux       = 0x0000; // ignored for SYS, since load address = $2000
	meta.mod_date  = nDateModify;
	meta.mod_time  = nTimeModify;
	//  .dir_block = TBD;

	return Util_ProDOS_AddFile( pDiskBytes, nDiskSize, pVolumeName, pFileData, nFileSize, meta, true );
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// bSectorsUsed is 16-bits bitmask of sectors.
// 1 = Free
// 0 = Used
//===========================================================================
void Util_DOS33_SetTrackSectorUsage ( uint8_t *pVTOC, const int nTrack, const uint16_t bSectorsFree )
{
	int nOffset = 0x38 + (nTrack * 4);

	// Byte  Sectors
	//    0  FEDC BA98
	//    1  7654 3210
	//    2  -Wasted-
	//    3  -Wasted-
#if _DEBUG
	std::string sFree;
	std::string sUsed;
	for( int iSector = 15; iSector >= 0; iSector-- )
	{
		bool bFree = (bSectorsFree >> iSector) & 1;
		sFree.append( 1, (char)( bFree) | '0' );
		sUsed.append( 1, (char)(!bFree) ^ '0' );
		if ((iSector & 3) == 0)
		{
			sFree.append( 1, ' ' );
			sUsed.append( 1, ' ' );
		}
	}
	LogOutput("Track: %02X, VTOC[ %02X ]: %02X %02X %02X %02X -> %02X %02X,  Free: %s, Used: %s\n"
		, nTrack
		, nOffset
		, pVTOC[ nOffset + 0 ]
		, pVTOC[ nOffset + 1 ]
		, pVTOC[ nOffset + 2 ]
		, pVTOC[ nOffset + 3 ]
		, (bSectorsFree >> 8) & 0xFF
		, (bSectorsFree >> 0) & 0xFF
		, sFree.c_str()
		, sUsed.c_str()
	);
#endif
	pVTOC[ nOffset + 0 ] = (bSectorsFree >> 8) & 0xFF;
	pVTOC[ nOffset + 1 ] = (bSectorsFree >> 0) & 0xFF;
	pVTOC[ nOffset + 2 ] = 0x00;
	pVTOC[ nOffset + 3 ] = 0x00;
}

//===========================================================================
void Util_DOS33_SetTrackUsed ( uint8_t *pDiskBytes, const int nVTOC_Track, int nTrackUsed )
{
	int nOffset = Util_GetTrackSectorOffset( nVTOC_Track, 0 );
	uint8_t *pVTOC = &pDiskBytes[ nOffset ];
	Util_DOS33_SetTrackSectorUsage( pVTOC, nTrackUsed, 0x0000 );
}

//===========================================================================
void Util_DOS33_FormatFileSystem ( uint8_t *pDiskBytes, const size_t nDiskSize, const int nVTOC_Track )
{
	const int nTracks = nDiskSize / TRACK_DENIBBLIZED_SIZE;
	int nOffset;

	assert (nTracks <= TRACKS_MAX);

	// Update CATALOG next track/sector
	for( int iSector = 0xF; iSector > 1; iSector-- )
	{
		nOffset = Util_GetTrackSectorOffset( nVTOC_Track, iSector );
		pDiskBytes[ nOffset + 1 ] = nVTOC_Track;
		pDiskBytes[ nOffset + 2 ] = iSector - 1;
	}

	// Last sector in CATALOG has no link
	nOffset = Util_GetTrackSectorOffset( nVTOC_Track, 1 );
	pDiskBytes[ nOffset + 1 ] = 0;
	pDiskBytes[ nOffset + 2 ] = 0;

	// FTOC = 256 bytes
	//      - HeaderSize = 0x0C
	//                     0x00 Wasted byte
	//                     0x01 Track Next FTOC
	//                     0x02 Sector Next FTOC
	//                     0x03 Wasted byte
	//                     0x04 Wasted byte
	//                     0x05 Sector offset of file in this T/S
	//                     0x06 Sector offset of file in this T/S
	//                     0x07 Wasted byte
	//                     0x08 Wasted byte
	//                     0x09 Wasted byte
	//                     0x0A Wasted byte
	//                     0x0B Wasted byte
	//      / 2 bytes for next Track/Sector
	//      = 122 entries
	const uint8_t FTOC_ENTRIES = (256 - 12) / 2;

	nOffset = Util_GetTrackSectorOffset( nVTOC_Track, 0 );
	pDiskBytes[ nOffset +  0x1 ] = nVTOC_Track;            // CATALOG = T11
	pDiskBytes[ nOffset +  0x2 ] = 0xF;                    // CATALOG = S0F
	pDiskBytes[ nOffset +  0x3 ] = 0x3;                    // DOS 3.3
	pDiskBytes[ nOffset +  0x6 ] = DEFAULT_VOLUME_NUMBER; // Volume
	pDiskBytes[ nOffset + 0x27 ] = FTOC_ENTRIES;           // TrackSector pairs
	pDiskBytes[ nOffset + 0x30 ] = nVTOC_Track;            // Last Track Allocated
	pDiskBytes[ nOffset + 0x31 ] = 1;                      // Direction = +1
	pDiskBytes[ nOffset + 0x34 ] = nTracks;                // Number of Tracks based on image size
	pDiskBytes[ nOffset + 0x35 ] = 16;                     // 16 Sectors/Track
	pDiskBytes[ nOffset + 0x36 ] = 0x00;                   // 256 Bytes/Sector Lo
	pDiskBytes[ nOffset + 0x37 ] = 0x01;                   // 256 Bytes/Sector Hi

	uint8_t *pVTOC = &pDiskBytes[ nOffset ];
	for( int iTrack = 0; iTrack < nTracks; iTrack++ )
	{
		/**/ if (iTrack ==           0) Util_DOS33_SetTrackSectorUsage(pVTOC, iTrack, 0x0000); // Track T00 can NEVER be free due to stupid DOS 3.3 design (1 byte bug of `BNE` instead of `BPL`)
		else if (iTrack == nVTOC_Track) Util_DOS33_SetTrackSectorUsage(pVTOC, iTrack, 0x0000);
		else /*                      */ Util_DOS33_SetTrackSectorUsage(pVTOC, iTrack, 0xFFFF); // Tracks T01-T10, T12-T22 are free for use
	}
}
