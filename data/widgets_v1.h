/*
 * This is the first version of Genie display fw.
 * It has an angular meter -1 to 6 PSI (scaled x10) and 3 LED digits: ENG, ATF and BAT
 */

#define HPA_TO_DPSI (100 /* hPa to Pa */ * 0.000145038 /* Pa to PSI */ * 10 /* PSI to dPSI */)

DECLARE_WIDGET("Boost gauge",  display, GENIE_OBJ_ANGULAR_METER, 0 /* idx */, 0 /* min */, 70 /* max */,
               10 /* boost gauge starts from -1 PSI (-10 dPSI) */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_DPSI)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_DPSI));
DECLARE_WIDGET("Coolant temp", display, GENIE_OBJ_LED_DIGITS, 0, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 10));
DECLARE_WIDGET("ATF temp",     display, GENIE_OBJ_LED_DIGITS, 1, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
DECLARE_WIDGET("Battery volt", display, GENIE_OBJ_LED_DIGITS, 2, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, REM, REM_BATTERY_VOLTAGE), 10));

