/******EEPROM .h*****/ 

#ifndef FLASH_EEPROM_H 
#define FLASH_EEPROM_H 

#define FL_EERPOM_StartAddr ((u32) 0x08017000) // start from address after 94kbyte flash size 
//#define FL_EERPOM_StartAddr ((u32) 0x08014000) // start from address after 80kbyte flash size 
//#define FL_EEPROM_EndAddr ((u32) 0x08012200) 
#define FL_EEPROM_EndAddr ((u32)	0x08017800) // 2K data size 
#define FL_EEPROM_PageSize ((u16) 0x400) // 1K 

/* Page status definitions */ 
#define ERASED ((uint16_t)0xFFFF) /* PAGE is empty */ 
#define RECEIVE_DATA ((uint16_t)0xEEEE) /* PAGE is marked to receive data */ 
#define VALID_PAGE ((uint16_t)0x0000) /* PAGE containing valid data */ 
typedef enum 
{ 
  Data_8Bit =1, 
  Data_16Bit, 
  Data_32Bit 

} Data_typeDef; 

#endif 