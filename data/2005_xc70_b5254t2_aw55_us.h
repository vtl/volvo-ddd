/*
 * 2005 XC70 2.5T AW55
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_CAR(car, "2005 XC70, engine B5254T2, transmission AW55, US market", CAN_BPS_500K, CAN_BPS_125K);
  SET_CAR_PARAM(car, ack_cb, car_serialize_queries);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, ECM, "ECM", 0x7a, 0x01200021, CAN_HS);
  SET_MODULE_PARAM(car, ECM, ack_cb, module_serialize_queries);

  // that one is always 14.3...
  //  DECLARE_SENSOR(car, ECM, ECM_BATTERY_VOLTAGE,      "Battery voltage",      ARRAY(0xa6, 0x15, 0x85, 0x01), VALUE_FLOAT, sensor->value.v_float = in->data.bytes[5] * 0.07);
  DECLARE_SENSOR(car, ECM, ECM_COOLANT_TEMPERATURE,  "Coolant temperature",  ARRAY(0xa6, 0x10, 0xb8, 0x01), VALUE_FLOAT, (sensor->value.v_float = in->data.bytes[5] * 0.7 - 48));
  DECLARE_SENSOR(car, ECM, ECM_AMBIENT_AIR_PRESSURE, "Ambient air pressure", ARRAY(0xa6, 0x10, 0x05, 0x01), VALUE_INT, (sensor->value.v_int = in->data.bytes[5] * 5));
  DECLARE_SENSOR(car, ECM, ECM_BOOST_PRESSURE,       "Boost pressure",       ARRAY(0xa6, 0x10, 0xef, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * in->data.bytes[5] + in->data.bytes[6]) / 25.6 /*, print_frame("boost", in) */));
  SET_SENSOR_PARAM(car, ECM, ECM_BOOST_PRESSURE, update_interval, 100);
  DECLARE_SENSOR(car, ECM, ECM_INTAKE_AIR_TEMPERATURE, "Ambient air pressure", ARRAY(0xa6, 0x10, 0xaf, 0x01), VALUE_FLOAT, (sensor->value.v_float = in->data.bytes[5] * 0.75 - 48));

  // FIXME 0x10 0xef is a boost pressure, oil level is likely 0x10 0x07
  // DECLARE_SENSOR(car, ECM, ECM_OIL_LEVEL,            "Oil level",            ARRAY(0xa6, 0x10, 0xef, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * in->data.bytes[4] + in->data.bytes[5]) * 0.003));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, TCM, "TCM", 0x6e, 0x01200005, CAN_HS);
  DECLARE_SENSOR(car, TCM, TCM_ATF_TEMPERATURE,      "ATF temperature",      ARRAY(0xa5, 0x0c, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * in->data.bytes[6] + in->data.bytes[7])));
  // FIXME remove, match fn is now autoguessed
  //  SET_SENSOR_PARAM(car, TCM, TCM_ATF_TEMPERATURE, match_fn, match_a5_fn);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, DEM, "DEM", 0x1a, 0x01204001, CAN_HS);
  DECLARE_SENSOR(car, DEM, DEM_PUMP_CURRENT,         "Pump current",         ARRAY(0xa6, 0x00, 0x05, 0x01), VALUE_INT, (1 /* FIXME */));
  // FIXME DEM_SOLENOID_CURRENT comes with DEM_PUMP_CURRENT
  DECLARE_SENSOR(car, DEM, DEM_OIL_PRESSURE,         "Oil pressure",         ARRAY(0xa6, 0x00, 0x03, 0x01), VALUE_INT, (1 /* FIXME */));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, REM, "REM", 0x46, 0x00800401, CAN_LS);
  DECLARE_SENSOR(car, REM, REM_BATTERY_VOLTAGE,      "Battery voltage",      ARRAY(0xa6, 0xd0, 0xd4, 0x01), VALUE_FLOAT, (sensor->value.v_float = in->data.bytes[5] / 8.0));

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, SWM, "SWM", 0x00, 0x0131726c, CAN_LS);
  DECLARE_SENSOR(car, SWM, SWM_AUDIO_CONTROLS,       "Audio controls",      ARRAY(0), VALUE_INT, (sensor->value.v_int = in->data.bytes[7] & 0xf));
  SET_SENSOR_PARAM(car, SWM, SWM_AUDIO_CONTROLS, ack_cb, swm_audio_controls_cb);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, CCM, "CCM", 0x00, 0x02803008, CAN_LS);
  DECLARE_SENSOR(car, CCM, CCM_AMBIENT_LIGHT,        "Ambient light",       ARRAY(0), VALUE_INT, (sensor->value.v_int = !!(in->data.bytes[3] >> 4)));
  SET_SENSOR_PARAM(car, CCM, CCM_AMBIENT_LIGHT, ack_cb, ccm_ambient_light_cb);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

  DECLARE_MODULE(car, CEM, "CEM", 0x00, 0x03200408, CAN_LS);
  DECLARE_SENSOR(car, CEM, CEM_GEARBOX_POSITION,     "Gearbox position",    ARRAY(0), VALUE_INT, (sensor->value.v_int = (in->data.bytes[6] & 0x30) >> 4));
  SET_SENSOR_PARAM(car, CEM, CEM_GEARBOX_POSITION, ack_cb, cem_gearbox_position_cb);

