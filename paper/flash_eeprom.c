#include "stm32f10x.h"
#include "flash_eeprom.h"


u16 Flash_EEPROM_Write_Data( u32 Start_Address, u16 *data, u16 No_of_Data, u8 Data_format) 
{ 
  // u32 Write_Addr_Cnt=Start_Address; 
  u32 End_Of_Address=0; 
  FLASHStatus=FLASH_COMPLETE; 
  FLASH_EEPROM_ProgramStatus= PASSED; 

  FLASH_Unlock(); 

  /*Start EEPROM*/ 
  Flash_EEPROM_Addr= Start_Address; 
  End_Of_Address= Start_Address +(No_of_Data * Data_format) ; 
  /* Write page data*/ 
  while((Flash_EEPROM_Addr<End_Of_Address) ) 
  { 

    /* Verify if Address and Address+2 contents are 0xFFFFFFFF */ 
    // if ((*( vu32 *)Flash_EEPROM_Addr) == 0xFFFFFFFF) 
    //	{ 
    switch(Data_format) 
    { 
      case Data_8Bit :
	break; 
      case Data_16Bit : 

	FLASHStatus = FLASH_ProgramHalfWord(Flash_EEPROM_Addr, *data); 
	if (FLASHStatus != FLASH_COMPLETE) 
	{ 
	  return FLASHStatus; 
	} 
	Flash_EEPROM_Addr +=2; // point to next 2byte address 
	data++; 
	break; 
	case Data_32Bit : 
	FLASHStatus = FLASH_ProgramWord(Flash_EEPROM_Addr, *data); 
	if (FLASHStatus != FLASH_COMPLETE) 
	{ 
	return FLASHStatus; 
	} 
	Flash_EEPROM_Addr +=4; // point to next 4byte address 
	data++; 
	break; 
    } 
    /* 
    }	
    else 
    { 
    Flash_EEPROM_Addr +=4; // point to next 4byte address 
    } 
    */	
  } 
  FLASH_Lock(); 
  return FLASHStatus; 

}

/*************************** 
Function : Erase the internal Flash data 
Input :- start address	
- Number of page 
Output: return the FLASH status 

***************************/ 
u16 Flash_EEPROM_ErasePage( u32 Start_Address, u16 No_of_Size) 
{ 
  FLASH_Unlock(); 
  FLASHStatus=FLASH_COMPLETE; 
  FLASH_EEPROM_ProgramStatus= PASSED; 

  Flash_EEPROM_Nb_Of_Page= No_of_Size; // 
  /*Clear all the flash */ 
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR); 

  /*erase the page*/ 
  for(EraseCounter=0; EraseCounter<Flash_EEPROM_Nb_Of_Page; EraseCounter++) 
  { 
    FLASHStatus = FLASH_ErasePage(Start_Address +(FL_EEPROM_PageSize* EraseCounter)); 

    if (FLASHStatus != FLASH_COMPLETE) 
    { 
      return FLASHStatus; 
    } 
  } 

  FLASH_Lock(); 

  return FLASHStatus; 

} 
****************************************/ 
u16 Flash_EEPROM_Read_Data( u32 Start_Address, u16 *dest_data, u16 No_of_Data, u8 Data_format) 
{ 
  u32 End_Of_Address=0; 
  /* 
  u32 *EEPROM_32Bit_p = (u32 *)Start_Address; // pointer point to address	after 72kbyte flash size 
  u16 *EEPROM_16Bit_p = (u32 *)Start_Address; 
  u8 *EEPROM_8Bit_p = (u32 *)Start_Address; 
  */ 
  FLASHStatus=FLASH_COMPLETE; 
  FLASH_EEPROM_ProgramStatus= PASSED; 


  // FLASH_Unlock(); 
  /*Start EEPROM*/ 
  Flash_EEPROM_Addr= Start_Address; 
  End_Of_Address= Start_Address +(No_of_Data * Data_format) ; 
  while((Flash_EEPROM_Addr<End_Of_Address)) 
  { 
    switch(Data_format) 
    { 
      case Data_8Bit : *dest_data = *(u8 *)Flash_EEPROM_Addr ;	// 
      dest_data++	; 
      Flash_EEPROM_Addr+=1; 
      break; 


      case Data_16Bit : *dest_data = *(u16 *)Flash_EEPROM_Addr ; 
      dest_data++; 
      Flash_EEPROM_Addr+=2; 
      break; 


      case Data_32Bit : *dest_data = *(u32 *)Flash_EEPROM_Addr ; 
      dest_data++; 
      Flash_EEPROM_Addr+=4; 
      break; 
      default: break; 

    } 

  } 
  FLASH_Lock(); 
  return FLASHStatus; 

} 
///////////////////////////////////////////////////////////////////////////////// 
/**************************************** 
Function : format the internal Flash data 
Input :- start address	
- Number of page 
Output: return FLASHStatus; 


****************************************/ 
u16 Flash_EEPROM_Format( u32 Start_Address, u32 End_Address) 
{ 
  FLASH_Unlock(); 
  FLASHStatus=FLASH_COMPLETE; 
  FLASH_EEPROM_ProgramStatus= PASSED; 

  Flash_EEPROM_Nb_Of_Page= (End_Address- Start_Address)/FL_EEPROM_PageSize; // 
  if (Flash_EEPROM_Nb_Of_Page< 1) 
  { 
    Flash_EEPROM_Nb_Of_Page=1; 
  } 
  /*Clear all the flash */ 
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR); 

  /*erase the page*/ 
  for(EraseCounter=0; EraseCounter<Flash_EEPROM_Nb_Of_Page; EraseCounter++) 
  { 
    FLASHStatus = FLASH_ErasePage(Start_Address +(FL_EEPROM_PageSize* EraseCounter)); 

    if (FLASHStatus != FLASH_COMPLETE) 
    { 
    return FLASHStatus; 
    } 
  } 

  FLASH_Lock(); 

  return FLASHStatus; 




}
