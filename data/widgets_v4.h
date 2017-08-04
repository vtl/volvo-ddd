/*
 * This is the fourth version of Genie display fw.
 *
 */

// MAIN IMP
#define HPA_TO_DPSI (100 /* hPa to Pa */ * 0.000145038 /* Pa to PSI */ * 10 /* PSI to dPSI */)
DECLARE_WIDGET("Boost gauge",     display, 0, GENIE_OBJ_ANGULAR_METER, 0 /* idx */, 0 /* min */, 100 /* max */,	       
               0 /* boost gauge starts from 0 PSI (0 dPSI) */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_DPSI)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_DPSI));
DECLARE_WIDGET("Coolant temp",    display, 0, GENIE_OBJ_LED_DIGITS, 2, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 1)) * 10);
// FIXME offset zero to F
DECLARE_WIDGET("Coolant temp sign",     display, 0, GENIE_OBJ_USER_LED, 3, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE)));
DECLARE_WIDGET("ATF temp",        display, 0, GENIE_OBJ_LED_DIGITS, 26, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 1)) * 10);
DECLARE_WIDGET("ATF temp sign",   display, 0, GENIE_OBJ_USER_LED, 4, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE)));
DECLARE_WIDGET("Intake air temp", display, 0, GENIE_OBJ_LED_DIGITS, 27, 0, 65535, temp_c_to_f(get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE), 1)) * 10);
DECLARE_WIDGET("Intake air temp sign",   display, 0, GENIE_OBJ_USER_LED, 5, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE)));

// MAIN MET
#define HPA_TO_HBAR (100 /* hPa to Pa */ * 0.00001 /* Pa to BAR */ * 100 /* BAR to hBAR */)

DECLARE_WIDGET("Boost gauge",     display, 1, GENIE_OBJ_ANGULAR_METER, 1 /* idx */, 0 /* min */, 100 /* max */,
               0 /* boost gauge starts from 0 BAR */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_HBAR)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_HBAR));
DECLARE_WIDGET("Coolant temp",    display, 1, GENIE_OBJ_LED_DIGITS, 3, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 10));
DECLARE_WIDGET("Coolant temp sign",     display, 1, GENIE_OBJ_USER_LED, 6, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE)));
DECLARE_WIDGET("ATF temp",        display, 1, GENIE_OBJ_LED_DIGITS, 4, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
DECLARE_WIDGET("ATF temp sign",   display, 1, GENIE_OBJ_USER_LED, 7, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE)));
DECLARE_WIDGET("Intake air temp", display, 1, GENIE_OBJ_LED_DIGITS, 5, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE), 10));
DECLARE_WIDGET("Intake air temp sign",   display, 1, GENIE_OBJ_USER_LED, 8, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_INTAKE_AIR_TEMPERATURE)));

// ECM
DECLARE_WIDGET("Engine speed",  display, 2, GENIE_OBJ_LED_DIGITS, 6, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_ENGINE_SPEED), 1));
DECLARE_WIDGET("Engine boost",  display, 2, GENIE_OBJ_LED_DIGITS, 0, 0, 65535,
               0 /* boost gauge starts from 0 BAR */
               + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_HBAR)
               - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_HBAR));
DECLARE_WIDGET("Engine torque", display, 2, GENIE_OBJ_LED_DIGITS, 1, 0, 65535, get_sensor_abs_value(find_module_sensor_by_id(car, TCM, TCM_ENGINE_TORQUE), 1));
DECLARE_WIDGET("Torque sign",   display, 2, GENIE_OBJ_USER_LED, 0, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, TCM, TCM_ENGINE_TORQUE)));
DECLARE_WIDGET("Battery volt",  display, 2, GENIE_OBJ_LED_DIGITS, 7, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, REM, REM_BATTERY_VOLTAGE), 10));
DECLARE_WIDGET("A/C pressure",  display, 2, GENIE_OBJ_LED_DIGITS, 22, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AC_PRESSURE), 1));
DECLARE_WIDGET("A/C on",        display, 2, GENIE_OBJ_USER_LED, 11, 0, 1, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AC_COMP_ACTIVE), 1));
DECLARE_WIDGET("Fuel pressure", display, 2, GENIE_OBJ_LED_DIGITS, 23, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_FUEL_PRESSURE), 1));
DECLARE_WIDGET("STFT",          display, 2, GENIE_OBJ_LED_DIGITS, 24, 0, 65535, get_sensor_abs_value(find_module_sensor_by_id(car, ECM, ECM_STFT), 10));
DECLARE_WIDGET("STFT sign",     display, 2, GENIE_OBJ_USER_LED, 1, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_STFT)));
DECLARE_WIDGET("LTFT",          display, 2, GENIE_OBJ_LED_DIGITS, 25, 0, 65535, get_sensor_abs_value(find_module_sensor_by_id(car, ECM, ECM_LTFT), 10));
DECLARE_WIDGET("LTFT sign",     display, 2, GENIE_OBJ_USER_LED, 2, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, ECM, ECM_LTFT)));

// TCM
DECLARE_WIDGET("Selector",      display, 3, GENIE_OBJ_STRING, 0, 0, 0, get_sensor_value(find_module_sensor_by_id(car, TCM_LS, TCM_GEARBOX_POSITION_S), 1));
DECLARE_WIDGET("Current gear",  display, 3, GENIE_OBJ_STRING, 1, 0, 0, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_CURRENT_GEAR), 1));
DECLARE_WIDGET("Engine torque", display, 3, GENIE_OBJ_LED_DIGITS, 11, 0, 500, get_sensor_abs_value(find_module_sensor_by_id(car, TCM, TCM_ENGINE_TORQUE), 1));
DECLARE_WIDGET("Engine torque sign", display, 3, GENIE_OBJ_USER_LED, 9, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, TCM, TCM_ENGINE_TORQUE)));
DECLARE_WIDGET("Gear ratio",    display, 3, GENIE_OBJ_LED_DIGITS, 12, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_GEAR_RATIO), 1000));
DECLARE_WIDGET("ATF temp",      display, 3, GENIE_OBJ_LED_DIGITS, 17, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
DECLARE_WIDGET("ATF temp sign", display, 3, GENIE_OBJ_USER_LED, 10, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE)));
DECLARE_WIDGET("SLT current",   display, 3, GENIE_OBJ_LED_DIGITS, 8, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLT_CURRENT), 1));
DECLARE_WIDGET("SLS current",   display, 3, GENIE_OBJ_LED_DIGITS, 9, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLS_CURRENT), 1));
DECLARE_WIDGET("SLU current",   display, 3, GENIE_OBJ_LED_DIGITS, 10, 0, 1200, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_SLU_CURRENT), 1));
DECLARE_WIDGET("S1 status",     display, 3, GENIE_OBJ_LED, 0, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S1_STATUS), 1));
DECLARE_WIDGET("S2 status",     display, 3, GENIE_OBJ_LED, 1, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S2_STATUS), 1));
DECLARE_WIDGET("S3 status",     display, 3, GENIE_OBJ_LED, 2, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S3_STATUS), 1));
DECLARE_WIDGET("S4 status",     display, 3, GENIE_OBJ_LED, 3, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S4_STATUS), 1));
DECLARE_WIDGET("S5 status",     display, 3, GENIE_OBJ_LED, 4, 0, 1, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S5_STATUS), 1));

// DEM
DECLARE_WIDGET("FL speed",      display, 4, GENIE_OBJ_LED_DIGITS, 13, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_FRONT_LEFT_SPEED), 10));
DECLARE_WIDGET("FR speed",      display, 4, GENIE_OBJ_LED_DIGITS, 14, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_FRONT_RIGHT_SPEED), 10));
DECLARE_WIDGET("RL speed",      display, 4, GENIE_OBJ_LED_DIGITS, 15, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_REAR_LEFT_SPEED), 10));
DECLARE_WIDGET("RR speed",      display, 4, GENIE_OBJ_LED_DIGITS, 16, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_REAR_RIGHT_SPEED), 10));
DECLARE_WIDGET("Oil temp",      display, 4, GENIE_OBJ_LED_DIGITS, 18, 0, 65535, get_sensor_abs_value(find_module_sensor_by_id(car, DEM, DEM_OIL_TEMPERATURE), 10));
DECLARE_WIDGET("Oil pressure",  display, 4, GENIE_OBJ_LED_DIGITS, 19, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_OIL_PRESSURE), 100));
DECLARE_WIDGET("Solenoid current",  display, 4, GENIE_OBJ_LED_DIGITS, 20, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_SOLENOID_CURRENT), 1));
DECLARE_WIDGET("Pump current",  display, 4, GENIE_OBJ_LED_DIGITS, 21, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, DEM, DEM_PUMP_CURRENT), 1));
DECLARE_WIDGET("Oil temp sign", display, 4, GENIE_OBJ_USER_LED, 12, 0, 1, !get_sensor_value_sign(find_module_sensor_by_id(car, DEM, DEM_OIL_TEMPERATURE)));

// TXT
//DECLARE_WIDGET("Text",          display, 5, GENIE_OBJ_STRING, 2, 0, 0, (long)"hello world!");
