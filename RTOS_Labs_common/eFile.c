// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Students implement these functions in Lab 4
// Jonathan W. Valvano 12/27/25
// Solution to lab 4
#include <stdint.h>
#include <string.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include <stdio.h>

#define SUCCESS 0
#define FAIL 1

int OpenFlag=0;              // 0 means not initialized

#define MAXBLOCK 256         // largest block number, 
#define DATASIZE 512         // 512
struct aBlock{
  char data[DATASIZE];   // blockes are exactly 512 bytes
};                                
typedef struct aBlock BlockType;

#define NOT_OPEN 255
int WOpenFile;               // directory index of file open for writing (0 to 30)
BlockType WCurrentBlock;          // 512 bytes of RAM copy of block used during writing
unsigned long WBlockNum;          // which block is stored in WCurrentBlock

int ROpenFile;                    // directory index of file open for reading (0 to 30)
BlockType RCurrentBlock;          // 512 bytes
unsigned long RBlockNum;          // which block is stored in RCurrentBlock
unsigned long RByteCnt;           // which byte from block will be read next (0 to DATASIZE-1)
unsigned long RTotalByteCnt;      // which byte from file will be read next (0 to file size - 1)

unsigned long BlankBlock[128];     // 512-byte block used temporarily

struct Entry{                // size = 16 bytes/file
  char Name[8];              // file name, up to 7 characters
  unsigned long First;       // first block     ((Size/DATASIZE)+1 = number of blocks)
  unsigned long Size;        // number of bytes (Size%DATASIZE = bytes in last block)
};  
typedef struct Entry EntryType;

#define MAXFILES 30
struct aDirectory{
  EntryType File[MAXFILES];    // up to 30 files
};                                
typedef struct aDirectory DirectoryType;
#define NONE {0,0,0,0,0,0,0,0}         // blank name    
const DirectoryType BlankDirectory = { 
{ { NONE, 0, 0},    // first file
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, 
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, 
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, 
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, 
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, 
  { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}, { NONE, 0, 0}}, // 30th file
};

#define BLOCKFREE 1 // Block not used in FAT
#define FAT_EOF 0 // End of file
struct MetadataBlock {                                                                                                                                    
  unsigned char FAT[MAXBLOCK];       // File allocation table - each entry points to next block in file
  unsigned long FreeMap[MAXBLOCK/32]; // 256 bits, 1 for each block
  unsigned char padding[DATASIZE - MAXBLOCK - (MAXBLOCK/32)*4]; // To make sure it takes up a whole block
};
typedef struct MetadataBlock MetadataType;
MetadataType Metadata;

#define NUMDIRECTORIES 2
DirectoryType Directories[NUMDIRECTORIES]; // RAM copies of directories
DirectoryType* CurrentDirectory;   // pointer to curent directory loaded
unsigned long DCurrentEntry;      // current directory entry

// Sets the given bit in the free map, indicating that block is now free
void FreeMapSetBit(unsigned char bit){
  Metadata.FreeMap[bit/32] |= (1 << (bit%32));
}

// Clears the given bit in the free map, indicating that block is now taken
void FreeMapClearBit(unsigned char bit){
  Metadata.FreeMap[bit/32] &= ~(1 << (bit%32));
}

// Returns the value of a given bit in the free map
unsigned char FreeMapGet(unsigned char bit){
  return (Metadata.FreeMap[bit/32] >> (bit%32)) & 1;
}

// Allocate a free block, returns index of block allocated
unsigned char FreeMapAlloc(){
  for (int i = 0; i < MAXBLOCK/32; i++){
    if (Metadata.FreeMap[i]){
      unsigned char bit = 0;
      unsigned long word = Metadata.FreeMap[i];
      while (!(word & 1)){
        word >>= 1;
        bit++;
      }
      unsigned char block = i*32 + bit;
      FreeMapClearBit(block);
      Metadata.FAT[block] = FAT_EOF;
      return block;
    }
  }
  return FAIL; // Block 1 cannot be allocated (directory 0), so this indicates fail because disk is full
}

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
  if(OpenFlag){
    return SUCCESS; // already open
  }
  eDisk_Init(0);   // initialize hardware, drive 0
  OpenFlag = 1;
  WOpenFile = NOT_OPEN; // not open WCurrentBlock is unused
  ROpenFile = NOT_OPEN; // not open RCurrentBlock is unused
  CurrentDirectory = NULL; // directory not loaded
  return SUCCESS;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
unsigned short block;
  unsigned long old = OS_LockScheduler();
  if(!OpenFlag){
    OS_UnLockScheduler(old);
    return FAIL;   // not initialized
  }
  for (int i = 0; i < MAXBLOCK/32; i++){ // Format free bitmap
    Metadata.FreeMap[i] = 0xFFFFFFFF; // Set all free
  }
  Metadata.FreeMap[0] &= ~7; // Reserve first 3 blocks
  for (block = 0; block < MAXBLOCK; block++){ // Clear FAT (set all entries to 1)
    Metadata.FAT[block] = BLOCKFREE;
  }
  if(eDisk_WriteBlock((const BYTE *)&Metadata,0)){ // format FAT + FreeMap
    OS_UnLockScheduler(old);
    return FAIL;   // write block error
  }
  if(eDisk_WriteBlock((const BYTE *)&BlankDirectory,1)){ // format directory 0
    OS_UnLockScheduler(old);
    return FAIL;   // write block error
  }
  if(eDisk_WriteBlock((const BYTE *)&BlankDirectory,2)){ // format directory 1
    OS_UnLockScheduler(old);
    return FAIL;   // write block error
  }
  for(block=3; block<MAXBLOCK; block++){
    if(eDisk_WriteBlock((const BYTE *)BlankBlock,block)){
      OS_UnLockScheduler(old);
      return FAIL; // write byte error
    }
  }
  OS_UnLockScheduler(old);
  CurrentDirectory = NULL; // directory not loaded
  return SUCCESS;   // OK
}

// bring directories from flash into RAM
// Output: 0 if successful and 1 on failure (e.g., trouble reading from flash)
int FetchMetadata(void){
  if( eDisk_ReadBlock((BYTE *)&Metadata,0)){ // first block is FAT + FreeMap
    return FAIL; 
  } 
  if( eDisk_ReadBlock((BYTE *)&Directories[0],1)){ // second block is directory 0
    CurrentDirectory = NULL;
    return FAIL; 
  } 
  if( eDisk_ReadBlock((BYTE *)&Directories[1],2)){ // third block is directory 1
    CurrentDirectory = NULL;
    return FAIL; 
  } 
  return SUCCESS;
}

// save RAM-copy of directories out to flash
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int BackupMetadata(void){
  if( eDisk_WriteBlock((BYTE *)&Metadata,0)){ // first block is FAT + FreeMap
    return FAIL; 
  }
  if( eDisk_WriteBlock((BYTE *)&Directories[0],1)){ // second block is directory 0
    return FAIL; 
  } 
  if( eDisk_WriteBlock((BYTE *)&Directories[1],2)){ // third block is directory 1
    return FAIL; 
  } 
  return SUCCESS;
}

//---------- eFile_Mount-----------------
// Mount the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure
int eFile_Mount(void){ // initialize file system
  if(!OpenFlag){
    return FAIL; // not initialized
  }  
  if(FetchMetadata() == FAIL){
    return FAIL;        // problem fetching directory
  }
  CurrentDirectory = &Directories[0];  // default to directory 0
  return SUCCESS;
}

//---------- eFile_SelectDirectory-----------------
// Select working directory
// Input: directory ID (0 or 1 since there are two)
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_SelectDirectory(unsigned char dirID) {
  if (dirID > 1){
    return FAIL;
  }
  if (CurrentDirectory == NULL){
    if (FetchMetadata() == FAIL){
      return FAIL;
    }
  }
  CurrentDirectory = &Directories[dirID];
  return SUCCESS;
}

//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( const char name[]){  // create new file, make it empty 
int i; unsigned char first;
  if(!OpenFlag){
    return FAIL;          // not initialized
  }
  if(strlen(name)>7){
    return FAIL; // name too long
  }

  if(CurrentDirectory == NULL){      // read if not already in memory
    if(FetchMetadata() == FAIL){
      return FAIL;        // problem fetching directory
    }
  }
  i = 0;        // search for duplicate
  while(i<MAXFILES){
    if(strcmp(CurrentDirectory->File[i].Name, name)==0){
      return FAIL;   // file already exists
    }
    i++;
  }  
  i = 0;        // search for free directory entry spot
  while((i<MAXFILES)&&(CurrentDirectory->File[i].Name[0])){
    i++;
  }
  if(i==(MAXFILES)){
    return FAIL;   // full directory, up to 30 files
  }
  first = FreeMapAlloc();
  if(first == FAIL){
    return FAIL;   // problem allocating its first block, e.g., disk full
  }
  
  strcpy(CurrentDirectory->File[i].Name,name); 
  CurrentDirectory->File[i].First = first; 
  CurrentDirectory->File[i].Size = 0;  // empty file
  return BackupMetadata();    // restore directory back to flash
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen( const char name[]){      // open a file for writing 
int i; 
  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(WOpenFile != NOT_OPEN){
    return FAIL;   // already open
  }
  if(CurrentDirectory == NULL){ // load if not previously loaded
    if(FetchMetadata() == FAIL){
      return FAIL;   // problem fetching directory
    }
  }
  
  i = 0;        // search for matching filename, strcmp returns 0 if equal
  while((i<MAXFILES) && (strcmp(CurrentDirectory->File[i].Name,name))){
    i++;
  }
  if((i==MAXFILES)||(i==ROpenFile)){   // can't have the same file open for read and write
    return FAIL;   // file does not exist or already open for read
  }
  WOpenFile = i;
  WBlockNum = CurrentDirectory->File[i].First;
  while(Metadata.FAT[WBlockNum] != FAT_EOF){    // keep reading until find the last block
    WBlockNum = Metadata.FAT[WBlockNum];
  }
  if(eDisk_ReadBlock((BYTE *)&WCurrentBlock,WBlockNum)){    // fetch data block
    WOpenFile = NOT_OPEN;
    return FAIL;   // trouble reading a data block
  }
  return SUCCESS;   
}

//---------- eFile_Write-----------------
// Save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write( const char data){unsigned long newBlock;
  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(WOpenFile == NOT_OPEN){  
    return FAIL;   // not open
  }
  unsigned long currentSize = CurrentDirectory->File[WOpenFile].Size;
  if(currentSize > 0 && currentSize % DATASIZE == 0){ // this block full?
    newBlock = FreeMapAlloc();
    if(newBlock == FAIL){
      eDisk_WriteBlock((const BYTE *)&WCurrentBlock,WBlockNum); // save full block to disk
      WOpenFile = NOT_OPEN;       // disk full, close
      BackupMetadata();
      return FAIL;            // problem allocating next block
    }
    Metadata.FAT[WBlockNum] = newBlock; // link previous to new one
    if(eDisk_WriteBlock((const BYTE *)&WCurrentBlock,WBlockNum)){ // save full block to disk
      WOpenFile = NOT_OPEN;
      return FAIL;   //trouble writing a data block
    }
    WBlockNum = newBlock; // new one becomes current
    Metadata.FAT[WBlockNum] = FAT_EOF; // mark this block as the end
  }
  WCurrentBlock.data[currentSize % DATASIZE] = data; // save into RAM buffer
  CurrentDirectory->File[WOpenFile].Size++;
  return SUCCESS;  
}

//---------- eFile_WriteString-----------------
// Save at end of the open file
// Input: pointer to string to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WriteString(const char *pt){ int max=512; 
  while(*pt){
    if(eFile_Write(*pt)) return FAIL;   //trouble writing
    pt++;
    max--;
    if(max==0)return FAIL;   //buffer overflow
  }
  return SUCCESS;
}

//-----------------------eFile_WriteUDec-----------------------
// Write a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
// Variable format 1-10 digits with space before and no space after
int eFile_WriteUDec(uint32_t n){
  char eOutBuf[12];
  eOutBuf[11] = 0;
  int i=10;
  do{
    eOutBuf[i] = '0'+n%10;
    n = n/10;
    i--;
  }while(n);
  eOutBuf[i] = ' ';
  return eFile_WriteString(&eOutBuf[i]);
}

//-----------------------eFile_WriteSDec-----------------------
// Write a 32-bit number in signed decimal format
// Input: 32-bit number to be transferred
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
// Variable format 1-10 digits with space before and no space after
int eFile_WriteSDec(int32_t num){
  char eOutBuf[12]; 
  int32_t n;
  if(num<0){
    n = -num;
  } else{
    n = num;
  }
  eOutBuf[11] = 0;
  int i=10;
  do{
    eOutBuf[i] = '0'+n%10;
    n = n/10;
    i--;
  }while(n);
  if(num<0){
    eOutBuf[i] = '-';
  } else{
    eOutBuf[i] = ' ';
  }  
  eOutBuf[i-1] = ' ';
  return eFile_WriteString(&eOutBuf[i-1]);
}

//-----------------------eFile_WriteSFix2-----------------------
// Write a 32-bit number in signed fixed point format
// signed 32-bit with resolution 0.01
// range -999.99 to +999.99
// Input: signed 32-bit integer part of fixed point number
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
// Examples
//   72345 to " 723.45"  
//  -22100 to "-221.00"
//    -102 to "  -1.02" 
//      31 to "   0.31" 
// -100000 to " ***.**"  
int eFile_WriteSFix2(int32_t num){
  char eOutBuf[8];
  int32_t n;
  if((num>99999)||(num<-99999)){
     return eFile_WriteString(" ***.**");
  }
  if(num<0){
    n = -num;
    eOutBuf[0] = '-';
  } else{
    n = num;
    eOutBuf[0] = ' ';
  }
  if(n>9999){
    eOutBuf[1] = '0'+n/10000;
    n = n%10000;
    eOutBuf[2] = '0'+n/1000;
  } else{
    if(n>999){
      if(num<0){
        eOutBuf[0] = ' ';
        eOutBuf[1] = '-';
      } else {
        eOutBuf[1] = ' ';
      }
      eOutBuf[2] = '0'+n/1000;
    } else{
      if(num<0){
        eOutBuf[0] = ' ';
        eOutBuf[1] = ' ';
        eOutBuf[2] = '-';
      } else {
        eOutBuf[1] = ' ';
        eOutBuf[2] = ' ';
      }
    }
  }
  n = n%1000;
  eOutBuf[3] = '0'+n/100;
  n = n%100;
  eOutBuf[4] = '.';
  eOutBuf[5] = '0'+n/10;
  n = n%10;
  eOutBuf[6] = '0'+n;
  eOutBuf[7] = 0;
  return eFile_WriteString(eOutBuf);
}


//-----------------------eFile_WriteUFix2-----------------------
// Write a 32-bit number in signed fixed point format
// unsigned 32-bit with resolution 0.01
// range  0.00 to 999.99
// Input: unsigned 32-bit integer part of fixed point number
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
// Examples
//   72345 to " 723.45"  
//   22100 to " 221.00"
//     102 to "   1.02" 
//      31 to "   0.31" 
//  100000 to " ***.**"  
int eFile_WriteUFix2(uint32_t num){
  if(num>99999){
     return eFile_WriteString(" ***.**");
  }
  return eFile_WriteSFix2((int32_t) num);
}

//---------- eFile_WClose-----------------
// Close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
  if(!OpenFlag){
    return FAIL;     // not initialized
  }
  if(WOpenFile==NOT_OPEN){
    return FAIL;     // not open
  }
  WOpenFile = NOT_OPEN; // Now closed for writing
  if(eDisk_WriteBlock((const BYTE *)&WCurrentBlock,WBlockNum)){ // save full block to disk
    return FAIL;   // trouble writing a data block
  }
  return BackupMetadata();    // restore directory back to flash
}


//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( const char name[]){      // open a file for reading 
int i; 
  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(ROpenFile != NOT_OPEN){
    return FAIL;   // already open
  }
  if(CurrentDirectory == NULL){ // load if not previously loaded
    if(FetchMetadata() == FAIL){
      return FAIL;   // problem fetching directory
    }
  }
  i = 0;          // search for matching filename
  while((i < MAXFILES) && (strcmp(CurrentDirectory->File[i].Name,name))){
    i++;
  }
  if((i == MAXFILES)||(i == WOpenFile)){   // can't have the same file open for read and write
    return FAIL;   // file does not exist or is open for write
  }
  ROpenFile = i;
  RBlockNum = CurrentDirectory->File[i].First;
  if(eDisk_ReadBlock((BYTE *)&RCurrentBlock,RBlockNum)){  // fetch data block
    ROpenFile = NOT_OPEN;
    return 1;   // trouble reading a data block
  }                              
  RByteCnt = 0; // start at the top of the block
  RTotalByteCnt = 0; // start at beginning of file
  return SUCCESS;     
}
 
//---------- eFile_ReadNext-----------------
// Retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(ROpenFile == NOT_OPEN){
    return FAIL;   // not open
  }
  unsigned long currentSize = CurrentDirectory->File[ROpenFile].Size;
  if(RTotalByteCnt < currentSize && RByteCnt < DATASIZE){ // this block have data to read?
    *pt = RCurrentBlock.data[RByteCnt];
    RByteCnt++;
    RTotalByteCnt++;
    return SUCCESS; // We can keep reading from this block
  }
  if(Metadata.FAT[RBlockNum] == FAT_EOF){    // no more blocks
    return FAIL; // end of file
  }
  RBlockNum = Metadata.FAT[RBlockNum];   // need to read next block
  if(eDisk_ReadBlock((BYTE *)&RCurrentBlock,RBlockNum)){  // fetch data block
    ROpenFile = NOT_OPEN;
    return FAIL;   // trouble reading a data block
  }                              
  RByteCnt = 0; // start at the top of the block
  if(RTotalByteCnt < currentSize){ // this block have any data?
    *pt = RCurrentBlock.data[0];
    RByteCnt++;
    RTotalByteCnt++;
    return SUCCESS; 
  }
  return FAIL; // end of file
}
//---------- eFileReadNextWord-----------------
// Retreive 32-bit little endian word from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
uint32_t eFileReadNextWord(uint32_t *pt){char data; int status; *pt=0;
  for(int i=0; i<32; i=i+8){
    status = eFile_ReadNext(&data);
    if(status==0){
      (*pt) |= data<<i; // little endian
    }
    else return FAIL;
  }
  return SUCCESS;
}
//---------- eFile_RClose-----------------
// Close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(ROpenFile==NOT_OPEN){
    return FAIL;   // not open
  }
  ROpenFile = NOT_OPEN; // Now closed for reading
  return SUCCESS;
}


//---------- eFile_Delete-----------------
// Delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( const char name[]){  // remove this file 
int i; unsigned short blknum;

  if(!OpenFlag){
    return FAIL;   // not initialized
  }
  if(WOpenFile!=NOT_OPEN){
    return FAIL;     // can't delete a file, if one open for writing
  }
  if(CurrentDirectory == NULL){ // load if not previously loaded
    if(FetchMetadata() == FAIL){
      return FAIL;   // problem fetching directory
    }
  }
  i = 0;          // search for matching filename
  while((i<MAXFILES) && (strcmp(CurrentDirectory->File[i].Name , name))){
    i++;
  }
  if(i==MAXFILES){
    return FAIL;   // file doesn't exist
  }
  CurrentDirectory->File[i].Name[0] = 0;  // delete directory entry
  CurrentDirectory->File[i].Size = 0;  // empty file
  
  blknum = CurrentDirectory->File[i].First;
  if(blknum != BLOCKFREE){
    while(Metadata.FAT[blknum] != FAT_EOF){    // keep reading until find the last block
      FreeMapSetBit(blknum); // indicate this block is free now
      unsigned long new_blknum = Metadata.FAT[blknum];
      Metadata.FAT[blknum] = BLOCKFREE; // indicate block is free in FAT
      blknum = new_blknum; // Move to next block in file
    }
    FreeMapSetBit(blknum); // indicate this block is free now
    Metadata.FAT[blknum] = BLOCKFREE;
  }
  return BackupMetadata();    // restore directory back to flash
}                             


//---------- eFile_DOpen-----------------
// Open a (sub)directory, read into RAM
// Input: directory name is an ASCII string up to seven characters
//        (empty/NULL for root directory)
// Output: 0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_DOpen( const char name[]){ // open directory
  if(!OpenFlag){
    return FAIL;       // not initialized
  }
  if(CurrentDirectory == NULL){ // load if not previously loaded
    if(FetchMetadata() == FAIL){
      return FAIL;     // problem fetching directory
    }
  }  
  DCurrentEntry = 0;
  return SUCCESS;
}
  
//---------- eFile_DirNext-----------------
// Retreive directory entry from open directory
// Input: none
// Output: return file name and size by reference
//         0 if successful and 1 on failure (e.g., end of directory)
int eFile_DirNext( char *name[], unsigned long *size){  // get next entry
  if(!OpenFlag){
    return FAIL;       // not initialized
  }
  if(CurrentDirectory == NULL){ 
    return FAIL;       // not opened
  }  
  while(DCurrentEntry<MAXFILES){
    if(CurrentDirectory->File[DCurrentEntry].Name[0]){  // file exists, if name is nonzero
      *name = CurrentDirectory->File[DCurrentEntry].Name;
      *size = CurrentDirectory->File[DCurrentEntry].Size;
      DCurrentEntry++;
      return SUCCESS;
    }
    DCurrentEntry++;
  }
  return FAIL;  
}

//---------- eFile_DClose-----------------
// Close the directory
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_DClose(void){ // close the directory
  return SUCCESS;  // nothing to do here
}


//---------- eFile_Unmount-----------------
// Unmount and deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently mounted)
int eFile_Unmount(void){ 
  if(OpenFlag){
    if (WOpenFile != NOT_OPEN){
      eDisk_WriteBlock((const BYTE *)&WCurrentBlock, WBlockNum);
    }
    if (BackupMetadata() == FAIL){
      return FAIL;
    }
    OpenFlag = 0;    // closed
    WOpenFile = NOT_OPEN; // not open
    ROpenFile = NOT_OPEN; // not open
    CurrentDirectory = NULL; // directory not loaded
    return SUCCESS;  
  }
  return FAIL;          // error, because not open
}
