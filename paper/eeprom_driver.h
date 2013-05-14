#ifndef __EEPROM_DRIVER_H__
#define __EEPROM_DRIVER_H__

extern void eeprom_setup(void);
extern void eeprom_write(u8 addr, u8 val);
extern u8 eeprom_read(u8 addr);

#endif