/*
 * 2005 XC70 2.5T AW55
 */

const char *get_tcm_gear_string(struct car *car)
{
  int gear =
    peek_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S1_STATUS), 1 << 4) |
    peek_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S2_STATUS), 1 << 3) |
    peek_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S3_STATUS), 1 << 2) |
    peek_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S4_STATUS), 1 << 1) |
    peek_sensor_value(find_module_sensor_by_id(car, TCM, TCM_S5_STATUS), 1 << 0);

  switch (gear) {
  case 0b00000:
    return "  N\n";
  case 0b00101:
    return "  R\n";
  case 0b11100:
    return "  1\n";
  case 0b00100:
    return "  2\n";
  case 0b00110:
    return "  3\n";
  case 0b00010:
    return "  4\n";
  case 0b01010:
    return "  5\n";
  default:
    return "  ?\n";
  }
}

const char *get_gearbox_level_position_string(int gear)
{
  static const char *gear_string[] = {"  P\n", "  R\n", "  N\n", "  D\n"};
  return gear_string[gear & 0b11];
}

void car_init(struct car * car)
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_CAR(car, "2005 XC70, engine B5254T2, transmission AW55, US market", CAN_BPS_500K, CAN_BPS_125K);
  SET_CAR_PARAM(car, ack_cb, car_serialize_queries);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, ECM, "ECM", 0x7a, 0x01200021, CAN_HS, MULTIFRAME);
  SET_MODULE_PARAM(car, ECM, ack_cb, module_serialize_queries);

  DECLARE_SENSOR(car, ECM, ECM_COOLANT_TEMPERATURE,  "Coolant temperature",  ARRAY(0xa6, 0x10, 0xb8, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 0.75 - 48));
  DECLARE_SENSOR(car, ECM, ECM_AMBIENT_AIR_PRESSURE, "Ambient air pressure", ARRAY(0xa6, 0x10, 0x05, 0x01), VALUE_INT, (sensor->value.v_int = bytes[5] * 5));
  DECLARE_SENSOR(car, ECM, ECM_BOOST_PRESSURE,       "Boost pressure",       ARRAY(0xa6, 0x10, 0xef, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * bytes[5] + bytes[6]) / 25.6 /*, print_frame("boost", in) */));
  SET_SENSOR_PARAM(car, ECM, ECM_BOOST_PRESSURE, update_interval, 75);
  DECLARE_SENSOR(car, ECM, ECM_INTAKE_AIR_TEMPERATURE, "Ambient air temperature", ARRAY(0xa6, 0x10, 0xaf, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 0.75 - 48));
  DECLARE_SENSOR(car, ECM, ECM_AC_PRESSURE,          "A/C pressure",         ARRAY(0xa6, 0x10, 0x01, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 13.54 - 176));
  SET_SENSOR_PARAM(car, ECM, ECM_AC_PRESSURE, update_interval, 500);
  DECLARE_SENSOR(car, ECM, ECM_AC_COMP_ACTIVE,       "A/C compressor active", ARRAY(0xa6, 0x10, 0x02, 0x01), VALUE_INT, (sensor->value.v_int = bytes[5] & 1));
  DECLARE_SENSOR(car, ECM, ECM_ENGINE_SPEED,         "Engine RPM",           ARRAY(0xa6, 0x10, 0x93, 0x01), VALUE_INT, (sensor->value.v_int = (256 * bytes[5] + bytes[6]) / 4));
  SET_SENSOR_PARAM(car, ECM, ECM_ENGINE_SPEED, update_interval, 500);
  DECLARE_SENSOR(car, ECM, ECM_STFT,                 "STFT",                 ARRAY(0xa6, 0x10, 0x51, 0x01), VALUE_FLOAT, (sensor->value.v_float = ((256 * bytes[5] + bytes[6]) * 2.0) / 65535));
  SET_SENSOR_PARAM(car, ECM, ECM_STFT, update_interval, 500);
  DECLARE_SENSOR(car, ECM, ECM_LTFT,                 "LTFT",                 ARRAY(0xa6, 0x11, 0x4c, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256 * bytes[5] + bytes[6]) * 0.046875));
  DECLARE_SENSOR(car, ECM, ECM_FAN_DUTY,             "Engine fan duty",      ARRAY(0xa6, 0x12, 0x48, 0x01), VALUE_INT, (sensor->value.v_int = bytes[5] * 100 / 255));
  DECLARE_SENSOR(car, ECM, ECM_MISFIRE_COUNTER,      "Misfire counter",      ARRAY(0xa6, 0x10, 0xad, 0x01), VALUE_INT, (sensor->value.v_int = 256L * bytes[5] + bytes[6]));

  SET_MODULE_PARAM(car, ECM, sensor_default_update_interval, 250);
  DECLARE_SENSOR(car, ECM, ECM_FUEL_PRESSURE,        "Fuel pressure",        ARRAY(0xa6, 0x15, 0x7d, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256L * bytes[5] + bytes[6]) * 0.0724792480 - 69));
  DECLARE_SENSOR(car, ECM, ECM_FUEL_PUMP_DUTY,       "Fuel pump duty",       ARRAY(0xa6, 0x15, 0x83, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256L * bytes[5] + bytes[6]) * 0.0015287890625));
  DECLARE_SENSOR(car, ECM, ECM_TCV_DUTY,             "TCV duty",             ARRAY(0xa6, 0x10, 0x2d, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 191.25 / 255));
  DECLARE_SENSOR(car, ECM, ECM_THROTTLE_ANGLE,       "Throttle angle",       ARRAY(0xa6, 0x10, 0x4e, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 100 / 255.0));
  DECLARE_SENSOR(car, ECM, ECM_MAF,                  "Mass air flow",        ARRAY(0xa6, 0x10, 0x9a, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256L * bytes[5] + bytes[6]) * 0.1));
  DECLARE_SENSOR(car, ECM, ECM_VVT_IN_ANGLE,         "VVT inlet angle",      ARRAY(0xa6, 0x13, 0x63, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256L * bytes[5] + bytes[6]) * 0.0390625));
  DECLARE_SENSOR(car, ECM, ECM_VVT_EX_ANGLE,         "VVT exhaust angle",    ARRAY(0xa6, 0x13, 0x62, 0x01), VALUE_FLOAT, (sensor->value.v_float = (int16_t)(256L * bytes[5] + bytes[6]) * 0.0390625));
  DECLARE_SENSOR(car, ECM, ECM_BTDC,                 "BTDC",                 ARRAY(0xa6, 0x10, 0x2c, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 191.25 / 255));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, TCM, "TCM", 0x6e, 0x01200005, CAN_HS, MULTIFRAME);
  SET_MODULE_PARAM(car, TCM, ack_cb, module_serialize_queries);
  SET_MODULE_PARAM(car, TCM, sensor_default_update_interval, 250);
  DECLARE_SENSOR(car, TCM, TCM_ATF_TEMPERATURE,      "ATF temperature",      ARRAY(0xa5, 0x0c, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * bytes[6] + bytes[7])));
  SET_SENSOR_PARAM(car, TCM, TCM_ATF_TEMPERATURE, update_interval, 1000);
  DECLARE_SENSOR(car, TCM, TCM_S1_STATUS,            "S1 solenoid status",   ARRAY(0xa5, 0x06, 0x01),       VALUE_INT, (sensor->value.v_int = bytes[4]));
  DECLARE_SENSOR(car, TCM, TCM_S2_STATUS,            "S2 solenoid status",   ARRAY(0xa5, 0x07, 0x01),       VALUE_INT, (sensor->value.v_int = bytes[4]));
  DECLARE_SENSOR(car, TCM, TCM_S3_STATUS,            "S3 solenoid status",   ARRAY(0xa5, 0x20, 0x01),       VALUE_INT, (sensor->value.v_int = bytes[4]));
  DECLARE_SENSOR(car, TCM, TCM_S4_STATUS,            "S4 solenoid status",   ARRAY(0xa5, 0x21, 0x01),       VALUE_INT, (sensor->value.v_int = bytes[4]));
  DECLARE_SENSOR(car, TCM, TCM_S5_STATUS,            "S5 solenoid status",   ARRAY(0xa5, 0x22, 0x01),       VALUE_INT, (sensor->value.v_int = bytes[4]));
  DECLARE_SENSOR(car, TCM, TCM_SLT_CURRENT,          "SLT solenoid current", ARRAY(0xa5, 0xb2, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * bytes[4] + bytes[5])));
  DECLARE_SENSOR(car, TCM, TCM_SLS_CURRENT,          "SLS solenoid current", ARRAY(0xa5, 0xb3, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * bytes[4] + bytes[5])));
  DECLARE_SENSOR(car, TCM, TCM_SLU_CURRENT,          "SLU solenoid current", ARRAY(0xa5, 0xb4, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * bytes[4] + bytes[5])));

  DECLARE_SENSOR(car, TCM, TCM_GEARBOX_POSITION,     "Gearbox position",    ARRAY(0xa5, 0x01, 0x01), VALUE_INT, (sensor->value.v_int = bytes[5] & 0x3));
  SET_SENSOR_PARAM(car, TCM, TCM_GEARBOX_POSITION, ack_cb, tcm_gearbox_position_cb);

// Engine torque and torque reduction come together
  DECLARE_SENSOR(car, TCM, TCM_ENGINE_TORQUE,        "Engine torque",        ARRAY(0xa5, 0x15, 0x01),       VALUE_INT,
                 (sensor->value.v_int = (int16_t)(256L * bytes[6] + bytes[7]),
                  (find_sensor_by_id(sensor->module, TCM_TORQUE_REDUCTION))->value.v_int = (int16_t)(256L * bytes[4] + bytes[5])
                 ));
  DECLARE_SENSOR(car, TCM, TCM_GEAR_RATIO,           "Gear ratio",           ARRAY(0xa5, 0x93, 0x01),       VALUE_FLOAT, (sensor->value.v_float = ((256L * bytes[4] + bytes[5]) == 0xffff ? 0 : (256L * bytes[4] + bytes[5])) * 0.001));
  DECLARE_SENSOR(car, TCM, TCM_CURRENT_GEAR,         "Current gear",         ARRAY(0x01),                   VALUE_STRING, (sensor->value.v_string = (const char *)get_tcm_gear_string(sensor->module->car)));
  DECLARE_SENSOR(car, TCM, TCM_GEARBOX_POSITION_S,   "Gearbox position S",   ARRAY(0x01), VALUE_STRING, (sensor->value.v_string = get_gearbox_level_position_string(peek_sensor_value(find_sensor_by_id(sensor->module, TCM_GEARBOX_POSITION), 1))));
  DECLARE_SENSOR(car, TCM, TCM_TORQUE_REDUCTION,     "Torque reduction",     ARRAY(0x00),                   VALUE_INT, (0));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, DEM, "DEM", 0x1a, 0x01204001, CAN_HS, MULTIFRAME);
  SET_MODULE_PARAM(car, DEM, ack_cb, module_serialize_queries);
  SET_MODULE_PARAM(car, DEM, sensor_default_update_interval, 250);
  // DEM_PUMP_CURRENT comes with DEM_SOLENOID_CURRENT
  DECLARE_SENSOR(car, DEM, DEM_PUMP_CURRENT,         "Pump current",         ARRAY(0xa6, 0x00, 0x05, 0x01), VALUE_INT,
                 (sensor->value.v_int = (int16_t)(256L * bytes[5] + bytes[6]),
                  ((find_sensor_by_id(sensor->module, DEM_SOLENOID_CURRENT))->value.v_int = (int16_t)(256L * bytes[7] + bytes[8]))));
  DECLARE_SENSOR(car, DEM, DEM_OIL_PRESSURE,         "Oil pressure",         ARRAY(0xa6, 0x00, 0x03, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] * 0.0164));
  DECLARE_SENSOR(car, DEM, DEM_OIL_TEMPERATURE,      "Oil temperature",      ARRAY(0xa6, 0x00, 0x02, 0x01), VALUE_INT, (sensor->value.v_int = (signed char)bytes[5]));
  DECLARE_SENSOR(car, DEM, DEM_FRONT_LEFT_SPEED,     "FL velocity",          ARRAY(0xa6, 0x00, 0x06, 0x01), VALUE_FLOAT,
                 (sensor->value.v_float                                                    = (uint16_t)(256 * bytes[ 8] + bytes[ 9]) * 0.0156,
                 (find_sensor_by_id(sensor->module, DEM_FRONT_RIGHT_SPEED))->value.v_float = (uint16_t)(256 * bytes[ 6] + bytes[ 7]) * 0.0156,
                 (find_sensor_by_id(sensor->module, DEM_REAR_LEFT_SPEED))->value.v_float   = (uint16_t)(256 * bytes[12] + bytes[13]) * 0.0156,
                 (find_sensor_by_id(sensor->module, DEM_REAR_RIGHT_SPEED))->value.v_float  = (uint16_t)(256 * bytes[10] + bytes[11]) * 0.0156));
  DECLARE_SENSOR(car, DEM, DEM_FRONT_RIGHT_SPEED,    "FR velocity",          ARRAY(0x00),                   VALUE_FLOAT, (0));
  DECLARE_SENSOR(car, DEM, DEM_REAR_LEFT_SPEED,      "RL velocity",          ARRAY(0x00),                   VALUE_FLOAT, (0));
  DECLARE_SENSOR(car, DEM, DEM_REAR_RIGHT_SPEED,     "RR velocity",          ARRAY(0x00),                   VALUE_FLOAT, (0));
  DECLARE_SENSOR(car, DEM, DEM_SOLENOID_CURRENT,     "Solenoid current",     ARRAY(0x00),                   VALUE_INT, (0));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, REM, "REM", 0x46, 0x00800401, CAN_LS, MULTIFRAME);
  DECLARE_SENSOR(car, REM, REM_BATTERY_VOLTAGE,      "Battery voltage",      ARRAY(0xa6, 0xd0, 0xd4, 0x01), VALUE_FLOAT, (sensor->value.v_float = bytes[5] / 8.0));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, SWM, "SWM", 0x00, 0x0131726c, CAN_LS, UNIFRAME);
  DECLARE_SENSOR(car, SWM, SWM_AUDIO_CONTROLS,       "Audio controls",      ARRAY(0), VALUE_INT, (sensor->value.v_int = bytes[7] & 0xf));
  SET_SENSOR_PARAM(car, SWM, SWM_AUDIO_CONTROLS, ack_cb, swm_audio_controls_cb);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, CEM, "CEM", 0x00, 0x02803008, CAN_LS, UNIFRAME);
  DECLARE_SENSOR(car, CEM, CEM_AMBIENT_LIGHT,        "Ambient light",       ARRAY(0), VALUE_INT, (sensor->value.v_int = !!(bytes[3] >> 4)));
  SET_SENSOR_PARAM(car, CEM, CEM_AMBIENT_LIGHT, ack_cb, cem_ambient_light_cb);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, CCM, "CCM", 0x29, 0x00801001, CAN_LS, UNIFRAME);
  DECLARE_SENSOR(car, CCM, CCM_SWITCH_STATUS,        "Switch status",       ARRAY(0xa6, 0x00, 0x77, 0x01), VALUE_INT, (sensor->value.v_int = !!(bytes[5] >> 4)));
  SET_SENSOR_PARAM(car, CCM, CCM_SWITCH_STATUS, ack_cb, ccm_switch_status_cb);
  SET_SENSOR_PARAM(car, CCM, CCM_SWITCH_STATUS, update_interval, 100);
  DECLARE_SENSOR(car, CCM, CCM_EVAP_TEMP,           "Evaporator temperature",ARRAY(0xa6, 0x00, 0x01, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * bytes[5] + bytes[6]) * 0.015625 - 100));
  DECLARE_SENSOR(car, CCM, CCM_CABIN_TEMP,          "Cabin temperature",    ARRAY(0xa6, 0x00, 0xa1, 0x01),  VALUE_FLOAT, (sensor->value.v_float = (256 * bytes[5] + bytes[6]) * 0.015625 - 100));
  DECLARE_SENSOR(car, CCM, CCM_BLOWER_DUTY,         "Cabin fan speed",      ARRAY(0xa6, 0x00, 0x30, 0x01),  VALUE_FLOAT, (sensor->value.v_float = (256 * bytes[5] + bytes[6]) * 0.015625));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
