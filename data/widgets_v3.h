/*
 * This is the third version of Genie display fw.
 * 
 */

// MAIN IMP
#define HPA_TO_DPSI (100 /* hPa to Pa */ * 0.000145038 /* Pa to PSI */ * 10 /* PSI to dPSI */)
DECLARE_WIDGET("Boost gauge",  display, 0, GENIE_OBJ_ANGULAR_METER, 0 /* idx */, 0 /* min */, 100 /* max */,
               0 /* boost gauge starts from 0 PSI (0 dPSI) */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_DPSI)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_DPSI));
DECLARE_WIDGET("Coolant temp", display, 0, GENIE_OBJ_LED_DIGITS, 0, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 1)) * 10);
DECLARE_WIDGET("ATF temp",     display, 0, GENIE_OBJ_LED_DIGITS, 1, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 1)) * 10);
DECLARE_WIDGET("Intake air temp", display, 0, GENIE_OBJ_LED_DIGITS, 2, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE), 1)) * 10);

// MAIN MET
#define HPA_TO_HBAR (100 /* hPa to Pa */ * 0.00001 /* Pa to BAR */ * 100 /* BAR to hBAR */)

DECLARE_WIDGET("Boost gauge",  display, 1, GENIE_OBJ_ANGULAR_METER, 1 /* idx */, 0 /* min */, 90 /* max */,
               0 /* boost gauge starts from 0 BAR */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_HBAR)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_HBAR));
DECLARE_WIDGET("Coolant temp", display, 1, GENIE_OBJ_LED_DIGITS, 3, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 10));
DECLARE_WIDGET("ATF temp",     display, 1, GENIE_OBJ_LED_DIGITS, 4, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
DECLARE_WIDGET("Intake air temp", display, 1, GENIE_OBJ_LED_DIGITS, 5, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE), 10));

// ECM
DECLARE_WIDGET("Battery volt", display, 2, GENIE_OBJ_LED_DIGITS, 7, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, REM, REM_BATTERY_VOLTAGE), 10));

// TCM
DECLARE_WIDGET("ATF temp",     display, 3, GENIE_OBJ_LED_DIGITS, 17, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
DECLARE_WIDGET("S1 status",    display, 3, GENIE_OBJ_LED, 0, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S1_STATUS), 1));
DECLARE_WIDGET("S2 status",    display, 3, GENIE_OBJ_LED, 1, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S2_STATUS), 1));
DECLARE_WIDGET("S3 status",    display, 3, GENIE_OBJ_LED, 2, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S3_STATUS), 1));
DECLARE_WIDGET("S4 status",    display, 3, GENIE_OBJ_LED, 3, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S4_STATUS), 1));
DECLARE_WIDGET("S5 status",    display, 3, GENIE_OBJ_LED, 4, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S5_STATUS), 1));

DECLARE_WIDGET("SLT current",  display, 3, GENIE_OBJ_LED_DIGITS, 8, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLT_CURRENT), 1));
DECLARE_WIDGET("SLS current",  display, 3, GENIE_OBJ_LED_DIGITS, 9, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLS_CURRENT), 1));
DECLARE_WIDGET("SLU current",  display, 3, GENIE_OBJ_LED_DIGITS, 10, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLU_CURRENT), 1));
