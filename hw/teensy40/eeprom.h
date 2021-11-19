#include <EEPROM.h>

#define EEPROM_CURRENT_SCREEN 0
#define EEPROM_CAN_POLL       1
#define EEPROM_RSE_LEFT_EN    2
#define EEPROM_RSE_RIGHT_EN   3
#define EEPROM_RTI_EN         4
#define EEPROM_KEY_CYCLE      5

void eeprom_init()
{
}

unsigned char eeprom_load(short int address, unsigned char def)
{
  unsigned char value;

  value = EEPROM.read(address);
  dprintf("EEPROM read address %d, got value %d\n", address, value);
  return value;
}

void eeprom_store(short int address, unsigned char value)
{
  dprintf("EEPROM write address %d value %d\n", address, value);
  EEPROM.write(address, value);
}
