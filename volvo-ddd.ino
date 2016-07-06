/*
   VOLVO Driver Distraction Display

   A car junkie display :)

   (c) 2016 Vitaly Mayatskikh <vitaly@gravicappa.info>
*/

int debug_print = 0;

#include <due_can.h>
#include <genieArduino.h>

#define LCD_RESETLINE 2

Genie genie;

/*
   known modules
*/
enum {
  ECM,
  TCM,
  DEM,
  REM
};

/*
   known sensors
*/
enum {
  ECM_BATTERY_VOLTAGE,
  ECM_COOLANT_TEMPERATURE,
  ECM_AMBIENT_AIR_PRESSURE,
  ECM_BOOST_PRESSURE,
  ECM_OIL_LEVEL
};

enum {
  TCM_ATF_TEMPERATURE
};

enum {
  DEM_PUMP_CURRENT,
  DEM_SOLENOID_CURRENT,
  DEM_OIL_PRESSURE
};

enum {
  REM_BATTERY_VOLTAGE
};

/*
   sensor value types
*/
enum {
  VALUE_INT,
  VALUE_FLOAT
};

struct sensor;
struct module;

typedef struct sensor {
  uint8_t id;
  char *name;
  struct module *module;
  union {
    long v_int;
    float v_float;
  } value;
  int value_type;
  int update_interval = 1000;
  long last_update;
  uint8_t request_size;
  uint8_t *request_data;
  bool (* match_fn)(struct sensor *, CAN_FRAME *);
  void (* convert_fn)(struct sensor *, CAN_FRAME *in);
  void (* ack_cb)(struct sensor *) = NULL;
  bool acked = true;
} sensor_t;

typedef struct module {
  uint8_t id;
  char *name;
  struct car *car;
  uint8_t req_id;
  uint32_t can_id;
  CANRaw *canbus;
  int update_interval = 0;
  long last_update;
  void (* ack_cb)(struct module *) = NULL;
  bool acked = true;
  int last_sensor = 0;
  uint8_t sensor_count = 0;
  sensor_t sensor[32];
} module_t;

typedef struct car {
  bool can_hs_ok = false;
  bool can_ls_ok = false;
  int can_hs_rate;
  int can_ls_rate;
  long last_update;
  bool acked = true;
  void (* ack_cb)(struct car *) = NULL;
  int module_count = 0;
  module_t module[16];
} car_t;

#define DECLARE_MODULE(_car, _id, _name, _req_id, _can_id, _canbus) \
  do { \
    struct module *module = &_car->module[_car->module_count]; \
    module->id = _id; \
    module->car = _car; \
    module->name = _name; \
    module->req_id = _req_id; \
    module->can_id = _can_id; \
    module->canbus = &_canbus; \
    module->sensor_count = 0; \
    _car->module_count++; \
  } while (0)

#define SET_MODULE_PARAM(_car, _id, _param, _value) \
  do { \
    struct module *module = find_module_by_id(_car, _id); \
    if (module) \
      module->_param = _value; \
  } while (0)

#define ARRAY(...) {__VA_ARGS__}

#define DECLARE_SENSOR(_car, _module_id, _id, _name, _req, _val_type, _fn)    \
  do {                  \
    struct module *module = find_module_by_id(_car, _module_id); \
    if (!module) \
      break; \
    struct sensor *sensor = &module->sensor[module->sensor_count]; \
    sensor->id = _id; \
    static uint8_t req_##_module_id_##_id[] = _req;     \
    sensor->request_data = req_##_module_id_##_id; \
    sensor->request_size = sizeof(req_##_module_id_##_id); \
    sensor->name = _name;     \
    sensor->module = &_car->module[_module_id]; \
    sensor->value_type = _val_type; \
    sensor->match_fn = sensor->request_data[0] == 0xa6 ? match_a6_fn : match_a5_fn; \
    sensor->convert_fn = [](struct sensor *sensor, CAN_FRAME *in) {/* print_frame(in); */_fn; }; \
    module->sensor_count++;       \
  } while (0)

#define SET_SENSOR_PARAM(_car, _module_id, _id, _param, _value) \
  do { \
    struct module *module = find_module_by_id(_car, _module_id); \
    if (!module) \
      break; \
    struct sensor *sensor = find_sensor_by_id(module, _id); \
    if (!sensor) \
      break; \
    sensor->_param = _value; \
  } while (0)

struct genie_display;

typedef struct genie_widget {
  char *name;
  struct genie_display *display;
  int object_type;
  int object_index;
  int last_value;
  int min_value;
  int max_value;
  int (*value_fn)(struct genie_widget *);
} genie_widget_t;

typedef struct genie_display {
  Genie genie;
  struct car *car;
  uint8_t widget_count = 0;
  genie_widget_t widget[32];
} genie_display_t;

void widget_update(struct genie_widget *widget)
{
  int new_value = widget->value_fn(widget);

  if (new_value < widget->min_value) {
    if (debug_print) {
      Serial.print("widget underflow for "); Serial.print(widget->name); Serial.print(": "); Serial.print(new_value); Serial.print(" < "); Serial.println(widget->min_value);
      new_value = widget->min_value;
    }
  }
  if (new_value < widget->max_value) {
    if (debug_print) {
      Serial.print("widget overflow for "); Serial.print(widget->name); Serial.print(": "); Serial.print(new_value); Serial.print(" > "); Serial.println(widget->min_value);
      new_value = widget->max_value;
    }
  }

  if (new_value != widget->last_value) {
    if (debug_print) {
      Serial.print("updating widget "); Serial.print(widget->name); Serial.print(" with value "); Serial.println(new_value);
    }
    long ms = millis();

    widget->display->genie.WriteObject(widget->object_type, widget->object_index, new_value);

    if (debug_print) {
      Serial.print("update took "); Serial.print(millis() - ms); Serial.println(" ms");
    }
    widget->last_value = new_value;
  }
}

#define DECLARE_WIDGET(_name, _display, _object_type, _object_index, _min, _max, _fn) \
  do { \
    struct genie_widget *widget = &_display->widget[_display->widget_count]; \
    widget->name = _name; \
    widget->display = _display; \
    widget->object_type = _object_type; \
    widget->object_index = _object_index; \
    widget->min_value = _min; \
    widget->max_value = _max; \
    widget->value_fn = [](struct genie_widget *widget)->int { struct car *car = widget->display->car; return (_fn); }; \
    widget->display->widget_count++;       \
  } while (0)

void reset_genie(Genie *genie)
{
  Serial.println("Resetting 4DSystems LCD...");
  genie->assignDebugPort(Serial);
  Serial2.begin(200000);
  genie->Begin(Serial2);
  pinMode(LCD_RESETLINE, OUTPUT);
  digitalWrite(LCD_RESETLINE, 0);
  delay(100);
  digitalWrite(LCD_RESETLINE, 1);

  delay (3500); //let the display start up after the reset (This is important)
}

void setup_genie_display(struct genie_display *display, struct car *car)
{
  display->car = car;
  display->widget_count = 0;
  reset_genie(&display->genie);

#define HPA_TO_DPSI (100 /* hPa to Pa */ * 0.000145038 /* Pa to PSI */ * 10 /* PSI to dPSI */)

  DECLARE_WIDGET("Boost gauge",  display, GENIE_OBJ_ANGULAR_METER, 0 /* idx */, 0 /* min */, 70 /* max */,
                 10 /* boost gauge starts from -1 PSI (-10 dPSI) */
                 + get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_BOOST_PRESSURE), HPA_TO_DPSI)
                 - get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_AMBIENT_AIR_PRESSURE), HPA_TO_DPSI));
  DECLARE_WIDGET("Coolant temp", display, GENIE_OBJ_LED_DIGITS, 0, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, ECM, ECM_COOLANT_TEMPERATURE), 10));
  DECLARE_WIDGET("ATF temp",     display, GENIE_OBJ_LED_DIGITS, 1, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, TCM, TCM_ATF_TEMPERATURE), 10));
  DECLARE_WIDGET("Battery volt", display, GENIE_OBJ_LED_DIGITS, 2, 0, 65535, get_sensor_value(find_module_sensor_by_id(car, REM, REM_BATTERY_VOLTAGE), 10));
}

void refresh_display(struct genie_display *display)
{
  for (int i = 0; i < display->widget_count; i++) {
    widget_update(&display->widget[i]);
  }
}

car_t my_car;
genie_display my_display;

void print_frame(char *s, CAN_FRAME *in)
{
  if (debug_print) {
    Serial.print(s);
    Serial.print(" ID = ");
    Serial.print(in->id, HEX);
    Serial.print(": ");
    for (int i = 0; i < 8; i++) {
      Serial.print(in->data.byte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

bool match_a6_fn(struct sensor *sensor, CAN_FRAME *in)
{
  //  Serial.println(sensor->request_data[0], HEX);
  //  Serial.println(sensor->request_data[1], HEX);
  //  Serial.println(sensor->request_data[2], HEX);
  return //(in->data.byte[2] == (sensor->request_data[0] | 0x40)) &&
    (in->data.byte[3] == sensor->request_data[1]) &&
    (in->data.byte[4] == sensor->request_data[2]);
}

bool match_a5_fn(struct sensor *sensor, CAN_FRAME *in)
{
  return (in->data.byte[2] == (sensor->request_data[0] | 0x40) &&
          in->data.byte[3] == sensor->request_data[1]);
}

struct module *find_module_by_id(struct car *car, int module_id)
{
  for (int m = 0; m < car->module_count; m++)
    if (car->module[m].id == module_id)
      return &car->module[m];
  if (debug_print) {
    Serial.print("can't find module by id "); Serial.println(module_id);
  }
  return NULL;
}

struct sensor *find_sensor_by_id(struct module *module, int sensor_id)
{
  for (int s = 0; s < module->sensor_count; s++)
    if (module->sensor[s].id == sensor_id)
      return &module->sensor[s];
  if (debug_print) {
    Serial.print(module->name); Serial.print(": can't find sensor by id "); Serial.println(sensor_id);
  }
  return NULL;
}

void module_serialize_queries(struct module *module)
{
  module->acked = true;
}

void car_serialize_queries(struct car *car)
{
  car->acked = true;
}

struct sensor *guess_sensor_by_reply(struct car *car, CAN_FRAME *in)
{
  for (int m = 0; m < car->module_count; m++) {
    if (in->id != car->module[m].can_id)
      continue;
    for (int s = 0; s < car->module[m].sensor_count; s++) {
      sensor_t *sensor = &car->module[m].sensor[s];
      if (sensor->match_fn(sensor, in))
        return sensor;
    }
  }

  return NULL;
}

void can_hs_callback(CAN_FRAME *in)
{
  sensor_t *sensor = guess_sensor_by_reply(&my_car, in);

  if (!sensor) {
    print_frame("H unknown", in);
    return;
  }

  if (debug_print) {
    Serial.print("got sensor ");
    Serial.print((long)(void *)sensor, HEX);
    Serial.print(" ");
    Serial.println(sensor->name);
  }

  if (sensor->convert_fn)
    sensor->convert_fn(sensor, in);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
}

void can_ls_callback(CAN_FRAME *in)
{
  sensor_t *sensor = guess_sensor_by_reply(&my_car, in);

  if (!sensor) {
    print_frame("L unknown", in);
    return;
  }

  if (debug_print) {
    Serial.print("got sensor ");
    Serial.print((long)(void *)sensor, HEX);
    Serial.print(" ");
    Serial.println(sensor->name);
  }

  if (sensor->convert_fn)
    sensor->convert_fn(sensor, in);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
}

void setup_canbus(struct car *car)
{
  module_t *module;
  int i;

  Serial.print("setup CAN-bus... ");

  car->can_hs_ok = Can0.begin(car->can_hs_rate);
  car->can_ls_ok = Can1.begin(car->can_ls_rate);

  for (i = 0; i < car->module_count; i++) {
    module = &car->module[i];
    module->canbus->setRXFilter(module->can_id, 0x1fffff, true);
  }

  Can0.setGeneralCallback(can_hs_callback);
  Can1.setGeneralCallback(can_ls_callback);

  Serial.print("CAN HS: "); Serial.println(car->can_hs_ok ? "done" : "failed");
  Serial.print("CAN LS: "); Serial.println(car->can_ls_ok ? "done" : "failed");
}

void setup() {
  Serial.begin(115200);
  Serial.println("start");
  setup_car_2005_xc70(&my_car);
  setup_canbus(&my_car);
  setup_genie_display(&my_display, &my_car);
}

void query_sensor(struct sensor *sensor)
{
  CAN_FRAME out;

  if (debug_print) Serial.println(sensor->name);

  memset(out.data.bytes, 0, 8);
  out.id = 0x0ffffe;
  out.extended = true;
  out.priority = 4;
  out.length = 8;

  out.data.byte[0] = 0xc8 + sensor->request_size + 1;
  out.data.byte[1] = sensor->module->req_id;
  for (int i = 0; i < sensor->request_size; i++)
    out.data.byte[2 + i] = sensor->request_data[i];
  //  print_frame("OUT", &out);
  sensor->module->canbus->sendFrame(out);
}

/*
   1 - can queue
   0 - can't queue
*/
bool query_module_next_sensor(struct module *module)
{
  /*
     something is wrong, module was not queried for 1 second. unblock and go
  */
  if ((millis() - module->last_update) > 1000)
    module->acked = true;
  /*
     if module is serialized and previous query is in flight, return "can't queue"
  */
  if (!module->acked)
    return false;

  /*
     if we poll module too often, return "can't queue"
  */
  if (module->update_interval &&
      ((millis() - module->last_update) < module->update_interval))
    return false;

  /*
     all sensor queried
  */
  if (module->last_sensor >= module->sensor_count) {
    module->last_sensor = 0;
    return false;
  }

  sensor_t *sensor = &module->sensor[module->last_sensor];

  /*
     if sensor is serialized and previous query is in flight, return "can't queue"
  */
  if (!sensor->acked)
    return false;

  module->last_sensor++;

  /*
     if sensor is passive (sending updates itself) then return "can queue"
  */
  if (sensor->request_size == 0)
    return true;

  /*
     if we poll sensor too often, return "can queue"
  */
  if (sensor->update_interval &&
      ((millis() - sensor->last_update) < sensor->update_interval))
    return true;

  module->car->last_update = module->last_update = sensor->last_update = millis();

  if (module->car->ack_cb)
    module->car->acked = false;
  if (module->ack_cb)
    module->acked = false;
  if (sensor->ack_cb)
    sensor->acked = false;

  query_sensor(sensor);

  return true;
}

void query_all_sensors(struct car *car)
{
  static int m = 0;

  /*
     something is wrong, car was not queried for 0.1 second. unblock and go
  */
  if ((millis() - car->last_update > 100))
    car->acked = true;

  if (!car->acked)
    return;

  if (m == car->module_count)
    m = 0;

  if (!query_module_next_sensor(&car->module[m]))
    m++;
}

long get_sensor_value(struct sensor *sensor, int multiplier)
{
  if (sensor->value_type == VALUE_INT)
    return sensor->value.v_int * multiplier;
  else
    return (int)round(sensor->value.v_float * multiplier);
}

struct sensor *find_module_sensor_by_id(struct car *car, int module_id, int sensor_id)
{
  struct module *module = find_module_by_id(car, module_id);

  if (!module)
    return NULL;

  return find_sensor_by_id(module, sensor_id);
}

void loop() {
  query_all_sensors(&my_car);
  refresh_display(&my_display);
}

void setup_car_2005_xc70(struct car *car)
{
  Serial.print("setup 2005 XC70... ");

  car->can_hs_rate = CAN_BPS_500K;
  car->can_ls_rate = CAN_BPS_125K;
  car->ack_cb = [](struct car * car) {
    car->acked = true;
  }; //car_serialize_queries;

  DECLARE_MODULE(car, ECM, "Engine control module (ECM)", 0x7a, 0x01200021, Can0);
  SET_MODULE_PARAM(car, ECM, ack_cb, module_serialize_queries);

  // that one is always 14.3...
  //  DECLARE_SENSOR(car, ECM, ECM_BATTERY_VOLTAGE,      "Battery voltage",      ARRAY(0xa6, 0x15, 0x85, 0x01), VALUE_FLOAT, sensor->value.v_float = in->data.bytes[5] * 0.07);
  DECLARE_SENSOR(car, ECM, ECM_COOLANT_TEMPERATURE,  "Coolant temperature",  ARRAY(0xa6, 0x10, 0xb8, 0x01), VALUE_FLOAT, (sensor->value.v_float = in->data.bytes[5] * 0.7 - 48));
  DECLARE_SENSOR(car, ECM, ECM_AMBIENT_AIR_PRESSURE, "Ambient air pressure", ARRAY(0xa6, 0x10, 0x05, 0x01), VALUE_INT, (sensor->value.v_int = in->data.bytes[5] * 5));
  DECLARE_SENSOR(car, ECM, ECM_BOOST_PRESSURE,       "Boost pressure",       ARRAY(0xa6, 0x10, 0xef, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * in->data.bytes[5] + in->data.bytes[6]) / 25.6 /*, print_frame("boost", in) */));
  SET_SENSOR_PARAM(car, ECM, ECM_BOOST_PRESSURE, update_interval, 100);

  // FIXME 0x10 0xef is a boost pressure, oil level is likely 0x10 0x07
  // DECLARE_SENSOR(car, ECM, ECM_OIL_LEVEL,            "Oil level",            ARRAY(0xa6, 0x10, 0xef, 0x01), VALUE_FLOAT, (sensor->value.v_float = (256 * in->data.bytes[4] + in->data.bytes[5]) * 0.003));

  DECLARE_MODULE(car, TCM, "Transmission control module (TCM)", 0x6e, 0x01200005, Can0);
  DECLARE_SENSOR(car, TCM, TCM_ATF_TEMPERATURE,      "ATF temperature",      ARRAY(0xa5, 0x0c, 0x01),       VALUE_INT, (sensor->value.v_int = (int16_t)(256L * in->data.bytes[6] + in->data.bytes[7])));
  // FIXME remove, match fn is now autoguessed
  //  SET_SENSOR_PARAM(car, TCM, TCM_ATF_TEMPERATURE, match_fn, match_a5_fn);

  DECLARE_MODULE(car, DEM, "DEM", 0x1a, 0x01204001, Can0);
  DECLARE_SENSOR(car, DEM, DEM_PUMP_CURRENT,         "Pump current",         ARRAY(0xa6, 0x00, 0x05, 0x01), VALUE_INT, (1 /* FIXME */));
  // FIXME DEM_SOLENOID_CURRENT comes with DEM_PUMP_CURRENT
  DECLARE_SENSOR(car, DEM, DEM_OIL_PRESSURE,         "Oil pressure",         ARRAY(0xa6, 0x00, 0x03, 0x01), VALUE_INT, (1 /* FIXME */));

  DECLARE_MODULE(car, REM, "Rear electronic module (REM)", 0x46, 0x00800401, Can1);
  DECLARE_SENSOR(car, REM, REM_BATTERY_VOLTAGE,      "Battery voltage",      ARRAY(0xa6, 0xd0, 0xd4, 0x01), VALUE_FLOAT, (sensor->value.v_float = in->data.bytes[5] / 8.0));
  Serial.println("done");
}

