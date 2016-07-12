/*
   VOLVO Driver Distraction Display

   A car junkie display :)

   (c) 2016 Vitaly Mayatskikh <vitaly@gravicappa.info>
*/

int debug_print = 0;

#define WIDGETS "data/widgets_v1.h"
#define CAR "data/2005_xc70_b5254t2_aw55_us.h"

#include <due_can.h>
#include <genieArduino.h>
#include <PrintEx.h>

#define LCD_RESETLINE 2

Genie genie;
StreamEx SerialEx = Serial;

#define CAN_HS Can0
#define CAN_LS Can1

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
  char *name;
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

#define DECLARE_CAR(_car, _name, _can_hs_rate, _can_ls_rate) \
  do { \
    _car->name = _name; \
    _car->can_hs_rate = _can_hs_rate; \
    _car->can_ls_rate = _can_ls_rate; \
  } while (0)

#define SET_CAR_PARAM(_car, _param, _value) \
  do { \
    struct car *car = _car; \
    car->_param = _value; \
  } while (0)

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
    sensor->convert_fn = [](struct sensor *sensor, CAN_FRAME *in) { _fn; }; \
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
    if (debug_print)
      SerialEx.printf("widget %s underflow: %d < %d\n", widget->name, new_value, widget->min_value);
    new_value = widget->min_value;
  } else if (new_value > widget->max_value) {
    if (debug_print)
      SerialEx.printf("widget %d overflow: %d > %d\n", widget->name, new_value, widget->max_value);
    new_value = widget->max_value;
  }

  if (new_value != widget->last_value) {
    if (debug_print)
      SerialEx.printf("updating widget %s %d -> %d\n", widget->name, widget->last_value, new_value);
    long ms = millis();

    widget->display->genie.WriteObject(widget->object_type, widget->object_index, new_value);

    if (debug_print)
      SerialEx.printf("widget %s update took %d ms\n", widget->name, millis() - ms);
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
  SerialEx.printf("Resetting 4DSystems LCD... ");
  genie->assignDebugPort(Serial);
  Serial2.begin(200000);
  genie->Begin(Serial2);
  pinMode(LCD_RESETLINE, OUTPUT);
  digitalWrite(LCD_RESETLINE, 0);
  delay(100);
  digitalWrite(LCD_RESETLINE, 1);

  delay (3500); //let the display start up after the reset (This is important)
  SerialEx.printf("done\n");
}

void setup_genie_display(struct genie_display *display, struct car *car)
{
  display->car = car;
  display->widget_count = 0;
  reset_genie(&display->genie);

#include WIDGETS
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
    SerialEx.printf("CAN_FRAME for %s ID 0xlx: ", s, in->id);
    for (int i = 0; i < 8; i++)
      SerialEx.printf("0x02x ", in->data.byte[i]);
    SerialEx.printf("\n");
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
  if (debug_print)
    SerialEx.printf("can't find module by id %d\n", module_id);

  return NULL;
}

struct sensor *find_sensor_by_id(struct module *module, int sensor_id)
{
  for (int s = 0; s < module->sensor_count; s++)
    if (module->sensor[s].id == sensor_id)
      return &module->sensor[s];
  if (debug_print)
    SerialEx.printf("%s can't find sensor by id %d\n", module->name, sensor_id);

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

void can_callback(CAN_FRAME *in)
{
  sensor_t *sensor = guess_sensor_by_reply(&my_car, in);

  if (!sensor) {
    print_frame("unknown CAN frame", in);
    return;
  }

  if (debug_print)
    SerialEx.printf("got sensor %s\n", sensor->name);

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

  SerialEx.printf("setup CAN-bus...\n");

  car->can_hs_ok = Can0.begin(car->can_hs_rate);
  car->can_ls_ok = Can1.begin(car->can_ls_rate);

  for (i = 0; i < car->module_count; i++) {
    module = &car->module[i];
    module->canbus->setRXFilter(module->can_id, 0x1fffff, true);
  }

  Can0.setGeneralCallback(can_callback);
  Can1.setGeneralCallback(can_callback);

  SerialEx.printf("CAN HS: %s\n", car->can_hs_ok ? "done" : "failed");
  SerialEx.printf("CAN LS: %s\n", car->can_ls_ok ? "done" : "failed");
}

void setup_car(struct car *car)
{
#include CAR
  SerialEx.printf("setup %s... done\n", car->name);
}

void setup() {
  Serial.begin(115200);
  SerialEx.printf("start\n");
  setup_car(&my_car);
  setup_canbus(&my_car);
  setup_genie_display(&my_display, &my_car);
}

void query_sensor(struct sensor *sensor)
{
  CAN_FRAME out;

  if (debug_print)
    SerialEx.printf("query %s\n", sensor->name);

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

