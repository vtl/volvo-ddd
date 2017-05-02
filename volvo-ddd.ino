/*
   VOLVO Driver Distraction Display

   A car junkie display :)

   (c) 2016 Vitaly Mayatskikh <vitaly@gravicappa.info>
*/

int debug_print = 1;

#define WIDGETS "data/widgets_v3.h"
#define CAR "data/2005_xc70_b5254t2_aw55_us.h"

#include <due_can.h>
#include <genieArduino.h>
#include <PrintEx.h>
#include <DueTimer.h>

float temp_c_to_f(float c)
{
  return c * 1.8 + 32;
}

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
  REM,
  SWM,
  CCM,
  CEM,
};

/*
   known sensors
*/
enum {
  ECM_BATTERY_VOLTAGE,
  ECM_COOLANT_TEMPERATURE,
  ECM_AMBIENT_AIR_PRESSURE,
  ECM_INTAKE_AIR_TEMPERATURE,
  ECM_BOOST_PRESSURE,
  ECM_AC_PRESSURE,
  ECM_FUEL_PRESSURE
};

enum {
  TCM_ATF_TEMPERATURE,
  TCM_S1_STATUS,
  TCM_S2_STATUS,
  TCM_S3_STATUS,
  TCM_S4_STATUS,
  TCM_S5_STATUS,
  TCM_SLT_CURRENT,
  TCM_SLS_CURRENT,
  TCM_SLU_CURRENT,
  TCM_ENGINE_TORQUE,
  TCM_TORQUE_REDUCTION,
  TCM_CURRENT_GEAR
};

enum {
  DEM_PUMP_CURRENT,
  DEM_SOLENOID_CURRENT,
  DEM_OIL_PRESSURE,
  DEM_OIL_TEMPERATURE
};

enum {
  REM_BATTERY_VOLTAGE
};

enum {
  SWM_AUDIO_CONTROLS
};

enum {
  CEM_GEARBOX_POSITION,
  CEM_GEARBOX_POSITION_S
};

enum {
  CCM_AMBIENT_LIGHT
};

/*
   sensor value types
*/
enum {
  VALUE_INT,
  VALUE_FLOAT,
  VALUE_STRING
};

struct sensor;
struct module;

typedef struct sensor {
  uint8_t id;
  const char *name;
  struct module *module;
  union {
    long v_int;
    float v_float;
    const char *v_string;
  } value;
  int value_type;
  int update_interval;
  long last_update;
  uint8_t request_size;
  uint8_t *request_data;
  bool (* match_fn)(struct sensor *, CAN_FRAME *);
  void (* convert_fn)(struct sensor *, unsigned char *, int);
  void (* ack_cb)(struct sensor *) = NULL;
  bool acked = true;
} sensor_t;

typedef struct module {
  uint8_t id;
  const char *name;
  struct car *car;
  uint8_t req_id;
  uint32_t can_id;
  CANRaw *canbus;
  int update_interval = 0;
  int sensor_default_update_interval = 1000;
  long last_update;
  long max_wait = 50;
  void (* ack_cb)(struct module *) = NULL;
  bool acked = true;
  int last_sensor = 0;
  uint8_t sensor_count = 0;
  sensor_t sensor[32];
} module_t;

typedef struct car {
  const char *name;
  bool can_poll = true;
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
    switch (sensor->request_data[0]) { \
      case 0xa6: sensor->match_fn = match_a6_fn; break; \
      case 0xa5: sensor->match_fn = match_a5_fn; break; \
      case 0xff: sensor->match_fn = NULL; break; \
      default:   sensor->match_fn = match_always_fn; break; \
    } \
    sensor->convert_fn = [](struct sensor *sensor, unsigned char *bytes, int len) { _fn; }; \
    sensor->update_interval = module->sensor_default_update_interval; \
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
  const char *name;
  struct genie_display *display;
  int screen;
  int object_type;
  int object_index;
  long last_value;
  long min_value;
  long max_value;
  long (*value_fn)(struct genie_widget *);
} genie_widget_t;

typedef struct genie_display {
  Genie genie;
  int current_screen = 0;
  struct car *car;
  uint8_t widget_count = 0;
  genie_widget_t widget[32];
} genie_display_t;

#define GENIE_OBJ_STRING (-1)

void widget_update(struct genie_widget *widget)
{
  long new_value = widget->value_fn(widget);

  if (widget->object_type != GENIE_OBJ_STRING) {
    if (new_value < widget->min_value) {
      if (debug_print)
        SerialEx.printf("widget %s underflow: %d < %d\n", widget->name, new_value, widget->min_value);
      new_value = widget->min_value;
    } else if (new_value > widget->max_value) {
      if (debug_print)
        SerialEx.printf("widget %s overflow: %d > %d\n", widget->name, new_value, widget->max_value);
      new_value = widget->max_value;
    }
  }
  if (new_value != widget->last_value) {
    if (debug_print)
      SerialEx.printf("updating widget %s %d -> %d\n", widget->name, widget->last_value, new_value);
    long ms = millis();

    if (widget->object_type == GENIE_OBJ_STRING) {
      if (debug_print)
        SerialEx.printf("string %d == '%s'\n", widget->object_index, (char *)new_value);
      widget->display->genie.WriteStr(widget->object_index, (char *)new_value);
    } else {
      widget->display->genie.WriteObject(widget->object_type, widget->object_index, new_value);
    }
    if (debug_print)
      SerialEx.printf("widget %s update took %d ms\n", widget->name, millis() - ms);
    widget->last_value = new_value;
  }
}

#define DECLARE_WIDGET(_name, _display, _screen, _object_type, _object_index, _min, _max, _fn) \
  do { \
    struct genie_widget *widget = &_display->widget[_display->widget_count]; \
    widget->name = _name; \
    widget->display = _display; \
    widget->screen = _screen; \
    widget->object_type = _object_type; \
    widget->object_index = _object_index; \
    widget->min_value = _min; \
    widget->max_value = _max; \
    widget->value_fn = [](struct genie_widget *widget)->long { struct car *car = widget->display->car; return (_fn); }; \
    widget->display->widget_count++;       \
  } while (0)

void reset_genie(Genie *genie)
{
  SerialEx.printf("Resetting 4DSystems LCD... ");
  genie->assignDebugPort(Serial);
  Serial2.begin(200000);
  genie->Begin(Serial2);
  pinMode(LCD_RESETLINE, OUTPUT);
  digitalWrite(LCD_RESETLINE, LOW);
  delay(100);
  digitalWrite(LCD_RESETLINE, HIGH);

  delay(5500); //let the display start up after the reset (This is important)
  SerialEx.printf("done\n");
}

void setup_genie_display(struct genie_display *display, struct car *car)
{
  display->car = car;
  display->widget_count = 0;
  reset_genie(&display->genie);

#include WIDGETS
}

void refresh_display(struct genie_display *display, int screen)
{
  if (screen != display->current_screen) {
    SerialEx.printf("changing screen to %d\n", screen);
    display->genie.WriteObject(GENIE_OBJ_FORM, screen, 0);
    display->current_screen = screen;
  }
  for (int i = 0; i < display->widget_count; i++) {
    if (display->widget[i].screen == display->current_screen)
      widget_update(&display->widget[i]);
  }
}

car_t my_car;
genie_display my_display;
int current_screen = 0;

void print_frame(const char *s, CAN_FRAME *in)
{
  if (debug_print) {
    SerialEx.printf("CAN_FRAME for %s ID 0x%lx: ", s, in->id);
    for (int i = 0; i < 8; i++)
      SerialEx.printf("0x%02x ", in->data.byte[i]);
    SerialEx.printf("\n");
  }
}

void dump_array(const unsigned char *p, int len)
{
  if (debug_print) {
    for (int i = 0; i < len; i++)
      SerialEx.printf("0x%02x ", p[i]);
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

bool match_always_fn(struct sensor *sensor, CAN_FRAME *in)
{
  return true;
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
    sensor->convert_fn(sensor, in->data.bytes, in->length);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
}

unsigned char can_data[256];

void can_callback1(CAN_FRAME *in)
{
  static sensor_t *sensor = NULL;
  static int idx = 0;
  int len;

  if (in->data.bytes[0] & 0x80) {
    sensor = guess_sensor_by_reply(&my_car, in);
    if (!sensor) {
      print_frame("unknown CAN frame", in);
      return;
    }
    idx = 0;
  }

  if (!sensor)
    return;

  len = in->length;
  if (idx + len >= sizeof(can_data)) {
    if (debug_print) {
      SerialEx.printf("dropped multiframe with len %d > %d\n", idx + len, sizeof(can_data));
      sensor = NULL;
      return;
    }
  }
  print_frame("in", in);
  SerialEx.printf("idx %d, len %d\n", idx, len);
  memcpy(can_data + idx, in->data.bytes, len);
  idx += len;

  if ((in->data.bytes[0] & 0x40) == 0) {
    return;
  }

  if (debug_print)
    SerialEx.printf("got sensor %s\n", sensor->name);

  if (debug_print)
    dump_array(can_data, idx);

  if (sensor->convert_fn)
    sensor->convert_fn(sensor, can_data, idx);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
  sensor = NULL;
  idx = 0;
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

#include CAR

void setup_car(struct car *car)
{
  car_init(car);
  SerialEx.printf("setup %s... done\n", car->name);
}

void query_sensor(struct sensor *sensor)
{
  CAN_FRAME out;

  if (debug_print)
    SerialEx.printf("query %s:%s\n", sensor->module->name, sensor->name);

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
  /* module sends updates on its own */
//  if (module->req_id == 0)
//    return false;
  /*
     something is wrong, module was not queried for too long. unblock and go
  */
  if ((millis() - module->last_update) > module->max_wait)
    module->acked = true;
  /*
     if module is serialized and previous query is in flight, return "can't queue"
  */
  if (!module->acked)
    return false;

  /*
     if we poll module too often, return "can't queue"
  */
  if (module->req_id && /* no poll */
      module->update_interval &&
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
  if (sensor->request_size == 1) {
    switch (sensor->request_data[0]) {
      case 1: sensor->convert_fn(sensor, NULL, 0); break;
      default: break;
    }
    return true;
  }
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

  if (!car->can_poll)
    return;
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

long get_sensor_value(struct sensor *sensor, float multiplier)
{
  if (sensor->value_type == VALUE_INT)
    return sensor->value.v_int * multiplier;
  else if (sensor->value_type == VALUE_FLOAT)
    return (int)round(sensor->value.v_float * multiplier);
  else
    return (long)sensor->value.v_string;
}

struct sensor *find_module_sensor_by_id(struct car *car, int module_id, int sensor_id)
{
  struct module *module = find_module_by_id(car, module_id);

  if (!module)
    return NULL;

  return find_sensor_by_id(module, sensor_id);
}

enum {
  RADIO_EVENT_SWC,
  RADIO_EVENT_ILLUMI,
  RADIO_EVENT_GEARBOX
};

struct radio;

struct radio_command {
  const char *name;
  int function;
  struct radio *radio;
  bool (*match_fn)(int);
  void (*fn)(struct radio_command *);
};

typedef struct radio {
  const char *name;
  void setup(struct car *car);
  void event(struct car *car, int event, int);
  int control_pin;
  int illumi_pin;
  int park_pin;
  int camera_pin;
  DueTimer *timer;
  bool busy;
  int cur_bit;
  uint8_t data[49];
  int commands;
  struct radio_command command[16];
} radio_t;

struct radio my_radio;

#define EVENT_HIT(x, mask) ((param & (mask)) == (x))
#define EVENT_MISS(x, mask) (!EVENT_HIT((x), (mask)))

#define DECLARE_RADIO_COMMAND(_name, _radio, _function, _match_fn, _fn) \
  do { \
    struct radio *radio = _radio; \
    struct radio_command *command = &radio->command[radio->commands]; \
    command->name = _name; \
    command->function = _function; \
    command->match_fn = [](int param)->bool { return _match_fn; }; \
    command->fn = [](struct radio_command *command) { _fn; }; \
    command->radio = radio; \
    radio->commands++; \
  } while (0)

#define N 100

void radio_send_bits(struct radio_command *command, const uint8_t bits[])
{
  struct radio *radio = command->radio;

  if (radio->busy)
    return;
  memcpy(radio->data, (const uint8_t[]) {
    0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0
  }, 25);
  memcpy(radio->data + 25, bits, 24);
  radio->busy = true;
  radio->cur_bit = 0;
  pinMode(radio->control_pin, OUTPUT);
  digitalWrite(radio->control_pin, !LOW);
  radio->timer->attachInterrupt(radio_isr_send_start).start(10000 - N);
}

void radio_isr_send_start()
{
  struct radio *radio = &my_radio;
  digitalWrite(radio->control_pin, !HIGH);
  radio->timer->attachInterrupt(radio_isr_send_bit).start(4500 - N);
}

void radio_isr_send_bit()
{
  struct radio *radio = &my_radio;

  digitalWrite(radio->control_pin, !radio->data[radio->cur_bit++]);
  radio->timer->attachInterrupt(radio_isr_send_bit_finish).start(1000 - N);
}

void radio_isr_send_bit_finish()
{
  struct radio *radio = &my_radio;

  digitalWrite(radio->control_pin, !HIGH);
  if (radio->cur_bit == 50) {
    //    radio->timer->attachInterrupt(radio_isr_stop).start(1000000);
    radio_isr_stop();
  } else
    radio->timer->attachInterrupt(radio_isr_send_bit).start(200 - N);
}

void radio_isr_stop()
{
  struct radio *radio = &my_radio;

  radio->timer->stop();
  radio->busy = false;
}

#define SWC_PIN     44
#define ILLUMI_PIN  42
#define PARK_PIN    40
#define CAMERA_PIN  38

void setup_radio(struct radio *radio)
{
  radio->timer = &Timer6;
  radio->busy = false;
  radio->commands = 0;
  radio->control_pin = SWC_PIN;
  radio->illumi_pin = ILLUMI_PIN;
  radio->park_pin = PARK_PIN;
  radio->camera_pin = CAMERA_PIN;

  pinMode(radio->control_pin, OUTPUT);
  digitalWrite(radio->control_pin, LOW);
  pinMode(radio->park_pin, OUTPUT);
  digitalWrite(radio->park_pin, HIGH);
  pinMode(radio->camera_pin, OUTPUT);
  digitalWrite(radio->camera_pin, HIGH);
  pinMode(radio->illumi_pin, OUTPUT);
  digitalWrite(radio->illumi_pin, LOW);

  DECLARE_RADIO_COMMAND("prev disk",  &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1010, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("next disk",  &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b0101, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("prev track", &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1110, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("next track", &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1101, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("vol down",   &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1011, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("vol up",     &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b0111, 0b1111), radio_send_bits(command, (const uint8_t[]) {
    0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0
  }));
  DECLARE_RADIO_COMMAND("next screen", &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b0110, 0b1111), radio_arm_next_screen());
  DECLARE_RADIO_COMMAND("next screen", &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1001, 0b1111), radio_toggle_canbus());

  // Volvo: 1 - day, 0 - night
  // Kenwood: +12v - dimmer on
  // ULN2003A or NPN transistor reverse value (0 - +12v, 1 - 0v)

  DECLARE_RADIO_COMMAND("dimmer on",  &my_radio, RADIO_EVENT_ILLUMI,  EVENT_HIT (0b0, 0b1), digitalWrite(command->radio->illumi_pin, LOW));
  DECLARE_RADIO_COMMAND("dimmer off", &my_radio, RADIO_EVENT_ILLUMI,  EVENT_MISS(0b1, 0b1), digitalWrite(command->radio->illumi_pin, HIGH));

  DECLARE_RADIO_COMMAND("P gear on",  &my_radio, RADIO_EVENT_GEARBOX, EVENT_HIT (0b01, 0b11), digitalWrite(command->radio->park_pin, HIGH));
  DECLARE_RADIO_COMMAND("P gear off", &my_radio, RADIO_EVENT_GEARBOX, EVENT_MISS(0b01, 0b11), digitalWrite(command->radio->park_pin, LOW));

  DECLARE_RADIO_COMMAND("R gear on",  &my_radio, RADIO_EVENT_GEARBOX, EVENT_HIT (0b10, 0b11), digitalWrite(command->radio->camera_pin, LOW));
  DECLARE_RADIO_COMMAND("R gear off", &my_radio, RADIO_EVENT_GEARBOX, EVENT_MISS(0b10, 0b11), digitalWrite(command->radio->camera_pin, HIGH));
}

static int screen_arm_busy = false;
static DueTimer *screen_arm_timer = &Timer7;

void radio_arm_next_screen()
{
  if (screen_arm_busy)
    return;

  screen_arm_busy = true;
  screen_arm_timer->attachInterrupt(radio_unarm_delay).start(1000000);
  current_screen = ++current_screen % 5;
}

void radio_unarm_delay()
{
  screen_arm_timer->stop();
  screen_arm_busy = false;
}

void radio_toggle_canbus()
{
  if (screen_arm_busy)
    return;

  screen_arm_busy = true;
  screen_arm_timer->attachInterrupt(radio_unarm_delay).start(1000000);

  my_car.can_poll = !my_car.can_poll;
}

void radio_event(struct radio *radio, int function, int param)
{
  for (int i = 0; i < radio->commands; i++) {
    struct radio_command *command = &radio->command[i];
    if ((command->function == function) && command->match_fn(param)) {
      if (debug_print)
        SerialEx.printf("key pressed: %s\n", command->name);
      command->fn(command);
    }
  }
}

void swm_audio_controls_cb(struct sensor *sensor)
{
  radio_event(&my_radio, RADIO_EVENT_SWC, get_sensor_value(sensor, 1));
}

void ccm_ambient_light_cb(struct sensor *sensor)
{
  radio_event(&my_radio, RADIO_EVENT_ILLUMI, get_sensor_value(sensor, 1));
}

void cem_gearbox_position_cb(struct sensor *sensor)
{
  radio_event(&my_radio, RADIO_EVENT_GEARBOX, get_sensor_value(sensor, 1));
}

void setup()
{
  Serial.begin(115200);
  SerialEx.printf("start\n");
  setup_car(&my_car);
  setup_radio(&my_radio);
  setup_canbus(&my_car);
  setup_genie_display(&my_display, &my_car);
}

void loop()
{
  while (1) {
    query_all_sensors(&my_car);
    refresh_display(&my_display, current_screen);
  }
}

