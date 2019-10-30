#include <Preferences.h>

Preferences preferences;

#define EEPROM_CURRENT_SCREEN "CUR_SCR"
#define EEPROM_CAN_POLL       "CAN_POLL"
#define EEPROM_RSE_LEFT_EN    "RSE_LEFT_EN"
#define EEPROM_RSE_RIGHT_EN   "RSE_RIGHT_EN"
#define EEPROM_RTI_EN         "RTI_EN"


void eeprom_init()
{
  preferences.begin("ddd", false);
}

unsigned char eeprom_load(const char *property, unsigned char def)
{
  unsigned char value;

  value = preferences.getUChar(property, def);
  dprintf("load property %s, got value %d\n", property, value);
  return value;
}

void eeprom_store(const char *property, unsigned char value)
{
  dprintf("store property %s value %d\n", property, value);
  preferences.putUChar(property, value);
}
