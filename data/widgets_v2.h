/*
 * This is the second version of Genie display fw.
 * It has an angular meter 0 to 10 PSI (scaled x10) and 3 LED digits: ENG, ATF and AIR (all in Fahrenheit)
 */

#define HPA_TO_DPSI (100 /* hPa to Pa */ * 0.000145038 /* Pa to PSI */ * 10 /* PSI to dPSI */)

DECLARE_WIDGET("Boost gauge",  display, GENIE_OBJ_ANGULAR_METER, 0 /* idx */, 0 /* min */, 100 /* max */,
               0 /* boost gauge starts from 0 PSI (0 dPSI) */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_DPSI)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_DPSI));
DECLARE_WIDGET("Coolant temp", display, GENIE_OBJ_LED_DIGITS, 0, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 1) * 10));
DECLARE_WIDGET("ATF temp",     display, GENIE_OBJ_LED_DIGITS, 1, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 1) * 10));
DECLARE_WIDGET("Intake air temp", display, GENIE_OBJ_LED_DIGITS, 2, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE), 1) * 10));

