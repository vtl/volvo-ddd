/*
   VOLVO Driver Distraction Display

   A car junkie display :)

   (c) 2016-2019 Vitaly Mayatskikh <vitaly@gravicappa.info>
*/

#include <PrintEx.h>

int debug_print = 0;
StreamEx SerialEx = Serial;//USB;
int pcv = 0;

//#define NO_CAN
//#define NO_CAN_CALLBACK /* poll vs irq */

#define WIDGETS "data/widgets.h"
#define CAR "data/2005_xc70_b5254t2_aw55_us.h"

#include <due_can.h>
#include <genieArduino.h>
#include <DueTimer.h>
#include <eeprom_93c86.h> // https://github.com/vtl/eeprom_93c86

#define __ASSERT_USE_STDERR
#include <assert.h>

float temp_c_to_f(float c)
{
  return c * 1.8 + 32;
}

#define LCD_RESET_LINE 2
#define LCD_RESET_DELAY 3500
#define LCD_CMD_TIMEOUT 750
#define WDT_TIMEOUT 1500

#define SWC_PIN     44
#define RTI_PIN     42
#define ILLUMI_PIN  40
#define CAMERA_PIN  38
#define GPS_BUT_PIN 36
#define GPS_PWR_PIN 34
#define PARK_PIN    32 // no op, no free wires in new design...
#define RSE_LEFT_DISPLAY_EN_PIN  45
#define RSE_RIGHT_DISPLAY_EN_PIN 43

#define EEPROM_MAGIC_0      0xAA
#define EEPROM_MAGIC_1      0x55
#define EEPROM_CURRENT_SCREEN 16
#define EEPROM_CAN_POLL       17
#define EEPROM_RSE_LEFT_EN    18
#define EEPROM_RSE_RIGHT_EN   19
#define EEPROM_RTI_EN         20
#define EEPROM_KEY_CYCLE      21

Genie genie;

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
  ECM_AC_COMP_ACTIVE,
  ECM_FUEL_PRESSURE,
  ECM_ENGINE_SPEED,
  ECM_STFT,
  ECM_LTFTL,
  ECM_LTFTM,
  ECM_LTFTH,
  ECM_FUEL_PUMP_DUTY,
  ECM_TCV_DUTY,
  ECM_THROTTLE_ANGLE,
  ECM_MAF,
  ECM_VVT_IN_ANGLE,
  ECM_VVT_EX_ANGLE,
  ECM_BTDC,
  ECM_FAN_DUTY,
  ECM_MISFIRE_COUNTER
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
  TCM_CURRENT_GEAR,
  TCM_GEAR_RATIO,
  TCM_GEARBOX_POSITION,
  TCM_GEARBOX_POSITION_S
};

enum {
  DEM_PUMP_CURRENT,
  DEM_SOLENOID_CURRENT,
  DEM_OIL_PRESSURE,
  DEM_OIL_TEMPERATURE,
  DEM_FRONT_LEFT_SPEED,
  DEM_FRONT_RIGHT_SPEED,
  DEM_REAR_LEFT_SPEED,
  DEM_REAR_RIGHT_SPEED
};

enum {
  REM_BATTERY_VOLTAGE
};

enum {
  SWM_AUDIO_CONTROLS
};

enum {
  CEM_AMBIENT_LIGHT
};

enum {
  CCM_SWITCH_STATUS,
  CCM_EVAP_TEMP,
  CCM_CABIN_TEMP,
  CCM_BLOWER_DUTY
};

/*
   sensor value types
*/
enum {
  VALUE_INT,
  VALUE_FLOAT,
  VALUE_STRING
};

enum {
  UNIFRAME,
  MULTIFRAME
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
  long last_used;
  uint8_t request_size;
  uint8_t *request_data;
  bool (* match_fn)(struct sensor *, CAN_FRAME *) = NULL;
  void (* convert_fn)(struct sensor *, unsigned char *, int) = NULL;
  void (* ack_cb)(struct sensor *) = NULL;
  bool acked = true;
} sensor_t;

#define MAX_SENSORS_PER_MODULE 32

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
  bool frame_type;
  int last_sensor = 0;
  uint8_t sensor_count = 0;
  uint8_t rcv_data[256];
  unsigned int rcv_idx = 0;
  sensor_t *rcv_sensor;
  sensor_t sensor[MAX_SENSORS_PER_MODULE];
} module_t;

#define MAX_MODULES_PER_CAR 32

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
  module_t module[MAX_MODULES_PER_CAR];
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

#define DECLARE_MODULE(_car, _id, _name, _req_id, _can_id, _canbus, _frame_type) \
  do { \
    struct module *module = &_car->module[_car->module_count]; \
    assert(_car->module_count < MAX_MODULES_PER_CAR); \
    module->id = _id;	\
    module->car = _car; \
    module->name = _name; \
    module->req_id = _req_id; \
    module->can_id = _can_id; \
    module->canbus = &_canbus; \
    module->frame_type = _frame_type; \
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
    assert(module->sensor_count < MAX_SENSORS_PER_MODULE); \
    sensor->id = _id; \
    static uint8_t req_##_module_id_##_id[] = _req;     \
    sensor->request_data = req_##_module_id_##_id; \
    sensor->request_size = sizeof(req_##_module_id_##_id); \
    sensor->name = _name;     \
    sensor->module = module; \
    sensor->value_type = _val_type; \
    switch (sensor->request_data[0]) { \
      case 0xa6: sensor->match_fn = match_a6_fn; break; \
      case 0xa5: sensor->match_fn = match_a5_fn; break; \
      case 0xff: sensor->match_fn = NULL; break; \
      default:   sensor->match_fn = match_always_fn; break; \
    } \
    sensor->convert_fn = [](struct sensor *sensor, unsigned char *bytes, int len) { _fn; }; \
    sensor->update_interval = module->sensor_default_update_interval; \
    sensor->last_used = millis(); \
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
  long last_value = -1;
  long min_value;
  long max_value;
  long current_value;
  long (*val_fn)(struct genie_widget *);
  void (*cb_fn)(struct genie_widget *);
} genie_widget_t;

#define MAX_WIDGETS 128

typedef struct genie_display {
  Genie genie;
  bool ready;
  long init_started_ms;
  bool enabled = true;
  int current_screen = 0;
  int max_screen = 0;
  struct car *car;
  int widget_count = 0;
  int light_level = 15;
  genie_widget_t widget[MAX_WIDGETS];
} genie_display_t;

#define GENIE_OBJ_STRING (-1)

bool widget_update(struct genie_widget *widget, bool force)
{
  long new_value;

  if (!widget->display->ready)
    return true;

  if (!widget->display->enabled)
    return true;

  if (widget->object_type == GENIE_OBJ_USERBUTTON)
    return true;

  new_value = widget->val_fn(widget);
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
  if (force || new_value != widget->last_value) {
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
    widget->last_value = new_value;
    
    if (debug_print)
      SerialEx.printf("widget %s update took %d ms\n", widget->name, millis() - ms);

    if (millis() - ms > LCD_CMD_TIMEOUT)
      return false;
  }
  return true;
}

#define DECLARE_WIDGET(_name, _display, _screen, _object_type, _object_index, _min, _max, _val_fn, _cb_fn) \
  do { \
    struct genie_widget *widget = &_display->widget[_display->widget_count]; \
    assert(_display->widget_count < MAX_WIDGETS); \
    widget->name = _name; \
    widget->display = _display; \
    widget->screen = _screen; \
    widget->object_type = _object_type; \
    widget->object_index = _object_index; \
    widget->min_value = _min; \
    widget->max_value = _max; \
    widget->val_fn = [](struct genie_widget *widget)->long { struct car *car = widget->display->car; return (_val_fn); }; \
    widget->cb_fn = [](struct genie_widget *widget)->void { struct car *car = widget->display->car; return (_cb_fn); }; \
    widget->display->widget_count++;       \
    widget->display->max_screen = max(widget->display->max_screen, _screen); \
  } while (0)

void reset_display(struct genie_display *display)
{
  SerialEx.printf("Resetting 4DSystems LCD... \n");
  Serial2.begin(200000);
  display->genie.Begin(Serial2);
  display->genie.AttachEventHandler(display_event_callback);
  display->ready = false;
  display->current_screen = 0;
  pinMode(LCD_RESET_LINE, OUTPUT);
  digitalWrite(LCD_RESET_LINE, LOW);
  delay(100);
  digitalWrite(LCD_RESET_LINE, HIGH);
  display->init_started_ms = millis();
}

car_t my_car;
genie_display my_display;
int current_screen;

void setup_genie_display(struct genie_display *display, struct car *car)
{
  display->car = car;
  display->widget_count = 0;
  reset_display(display);

#include WIDGETS
}

bool loop_now_update = true;
long loop_now = 0; // timestamp of loop()

void refresh_display(struct genie_display *display, int screen)
{
  bool force = false;
  static bool display_off_once = false;

  if (!display->ready) {
    if (millis() - display->init_started_ms > LCD_RESET_DELAY)
      display->ready = true;
    else
      return;
  }

  if (!display->enabled) {
    if (!display_off_once) {
          display->genie.WriteContrast(display->enabled ? display->light_level :0);
          display_off_once = true;
    }
    return;
  }

  if (display->ready) {
    display->genie.DoEvents();
    long ms = millis();
    display->genie.WriteContrast(display->enabled ? display->light_level : 0);

    if (millis() - ms > LCD_CMD_TIMEOUT) {
      reset_display(display);
      goto do_widgets;
    }

    if (screen != display->current_screen && screen >= 0 && screen <= display->max_screen) {
      if (debug_print)
        SerialEx.printf("changing screen from %d to %d\n", display->current_screen, screen);
      display->genie.WriteObject(GENIE_OBJ_FORM, screen, 0);
      display->current_screen = screen;
      force = true;
    }
  }
do_widgets:
  for (int i = 0; i < display->widget_count; i++) {
    if (display->widget[i].screen == display->current_screen) {
      if (!widget_update(&display->widget[i], force)) {
         reset_display(display);
      }
    }
  }
}

bool set_widget(struct genie_display *display, const char *name, long value)
{
  int i;

  for (i = 0; i < display->widget_count; i++) {
    if (strncmp(display->widget[i].name, name, sizeof(display->widget[i].name)) == 0) {
      display->widget[i].current_value = value;
      break;
    }
  }

  return i != display->widget_count;
}

void genie_set_screen(int screen)
{
  current_screen = screen;
  eeprom_store(EEPROM_CURRENT_SCREEN, current_screen);
}

void genie_change_screen(int offset)
{
  genie_set_screen((current_screen + offset) % (my_display.max_screen + 1));
}

void genie_next_screen()
{
  genie_change_screen(1);
}

void genie_previous_screen()
{
  genie_change_screen(my_display.max_screen);
}

void watchdog_reset() // should be called from interrupt handler
{
  long now = millis();

  // make sure main loop is alive

  if (now - loop_now < WDT_TIMEOUT)
    watchdogReset();
}

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

struct module *find_module_by_can_id(struct car *car, unsigned long can_id)
{
  for (int m = 0; m < car->module_count; m++)
    if (car->module[m].can_id == can_id)
      return &car->module[m];
  if (debug_print)
    SerialEx.printf("can't find module by can_id %d\n", can_id);

  return NULL;
}

struct sensor *find_sensor_by_id(struct module *module, int sensor_id)
{
  if (!module)
    return NULL;
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

  print_frame("CAN frame", in);
  if (debug_print)
    SerialEx.printf("got sensor %s:%s\n", sensor->module->name, sensor->name);

  if (sensor->convert_fn)
    sensor->convert_fn(sensor, in->data.bytes, in->length);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
  watchdog_reset();
}

void can_callback_multiframe(char *msg, CAN_FRAME *in)
{
  int offset;
  int len;
  struct sensor *sensor = NULL;
  struct module *module = NULL;

  sensor = guess_sensor_by_reply(&my_car, in);
  if (!sensor) {
//    print_frame("unknown CAN frame", in);
    return;
  }

  if (sensor->module->frame_type == UNIFRAME) {
    can_callback(in);
    return;
  }

  print_frame(msg, in);

  if (in->data.bytes[0] & 0x80) { // frame start
    module = sensor->module;
    module->rcv_sensor = sensor;
    module->rcv_idx = 0;
    offset = 0;
    len = in->length; // copy everything
  } else {
      offset = 1; // skip flags and len
      len = in->data.bytes[0] & 0x0f - 8;
  }

  module = find_module_by_can_id(&my_car, in->id);
  if (!module) {
    return;
  }

  sensor = module->rcv_sensor;

  if (!sensor)
    return;

  if (module->rcv_idx + len >= sizeof(module->rcv_data)) {
    if (debug_print) {
      SerialEx.printf("dropped multiframe with len %d > %d\n", module->rcv_idx + len, sizeof(module->rcv_data));
    }
    goto out;
  }

  if (offset > 8) {
    if (debug_print) {
      SerialEx.printf("offset %d > 8\n", offset);
    }
    goto out;
  }

  if (len < 0) {
    if (debug_print) {
      SerialEx.printf("len %d < 0\n", len);
    }
    goto out;
  }

  if (offset > len) {
    if (debug_print) {
      SerialEx.printf("offset %d > len %d\n", offset, len);
    }
    goto out;
  }

  if (debug_print)
    SerialEx.printf("idx %d, len %d\n", module->rcv_idx, len);
  memcpy(module->rcv_data + module->rcv_idx, in->data.bytes + offset, len - offset); // FIXME how many bytes to skip?
  module->rcv_idx += len - offset;

  if ((in->data.bytes[0] & 0x40) == 0) { // not frame end
    return;
  }

  if (debug_print)
    SerialEx.printf("got sensor %s\n", sensor->name);

  if (debug_print)
    dump_array(module->rcv_data, module->rcv_idx);

  if (sensor->convert_fn)
    sensor->convert_fn(sensor, module->rcv_data, module->rcv_idx);
  if (sensor->ack_cb)
    sensor->ack_cb(sensor);
  if (sensor->module->ack_cb)
    sensor->module->ack_cb(sensor->module);
  if (sensor->module->car->ack_cb)
    sensor->module->car->ack_cb(sensor->module->car);
  watchdog_reset();
out:
  module->rcv_sensor = NULL;
  module->rcv_idx = 0;
}

void can_callback0(CAN_FRAME *in)
{
  can_callback_multiframe((char *)"CAN HS", in);
}

void can_callback1(CAN_FRAME *in)
{
  can_callback_multiframe((char *)"CAN LS", in);
}

void check_can(const char *id, CANRaw *can)
{
  CAN_FRAME in;

  while (can->rx_avail()) {
    can->read(in);
    can_callback_multiframe((char *)id, &in);
  }
}

void check_cans(struct car *car)
{
  check_can("HS", &CAN_HS);
  check_can("LS", &CAN_LS);
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

#ifndef NO_CAN_CALLBACK
  Can0.setGeneralCallback(can_callback0);
  Can1.setGeneralCallback(can_callback1);
#endif

  SerialEx.printf("CAN HS: %s\n", car->can_hs_ok ? "done" : "failed");
  SerialEx.printf("CAN LS: %s\n", car->can_ls_ok ? "done" : "failed");
}

#include CAR

void setup_car(struct car *car)
{
  car_init(car);

  pinMode(RSE_LEFT_DISPLAY_EN_PIN, OUTPUT);
//  digitalWrite(RSE_LEFT_DISPLAY_EN_PIN, HIGH);
  pinMode(RSE_RIGHT_DISPLAY_EN_PIN, OUTPUT);
//  digitalWrite(RSE_RIGHT_DISPLAY_EN_PIN, HIGH);

  loop_now = millis();

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
  /*
     something is wrong, module was not queried for too long. unblock and go
  */
  if ((millis() - module->last_update) > module->max_wait)
    module->acked = true;
  if (!module->car->acked)
    return false;
  /*
     if module is serialized and previous query is in flight, return "can't queue"
  */
  if (module->ack_cb && !module->acked)
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
     something is wrong, sensor was not queried for too long. unblock and go
  */
  if ((millis() - sensor->last_update) > 2 * sensor->update_interval)
    sensor->acked = true;

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

  /*
   * if sensor value was not used in last 5 seconds or more
   */
  if (millis() > sensor->last_used + 5000)
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

  if (m >= car->module_count)
    m = 0;

  if (!query_module_next_sensor(&car->module[m])) /* one sensor per module per loop */
    m++;
  get_sensor_value(find_module_sensor_by_id(car, CCM, CCM_SWITCH_STATUS), 1); // FIXME no widgets need it, so keep it alive
}

long peek_sensor_value(struct sensor *sensor, float multiplier)
{
  if (!sensor)
    return 0;
  if (sensor->value_type == VALUE_INT)
    return sensor->value.v_int * multiplier;
  else if (sensor->value_type == VALUE_FLOAT)
    return (int)round(sensor->value.v_float * multiplier);
  else
    return (long)sensor->value.v_string;
}

long get_sensor_value(struct sensor *sensor, float multiplier)
{
  if (!sensor)
    return 0;
  sensor->last_used = millis();
  return peek_sensor_value(sensor, multiplier);
}

long get_sensor_abs_value(struct sensor *sensor, float multiplier)
{
  return abs(get_sensor_value(sensor, multiplier));
}

bool get_sensor_value_sign(struct sensor *sensor)
{
  return (get_sensor_value(sensor, 1) >= 0);
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

#define RADIO_EVENT_FIRST_REPEAT_DELAY_MS 1000 // delay between first event and following

struct radio;

struct radio_command {
  const char *name;
  int function;
  struct radio *radio;
  bool (*match_fn)(int);
  void (*fn)(struct radio_command *);
};

#define MAX_RADIO_COMMANDS 16

typedef struct radio {
  const char *name;
  void setup(struct car *car);
  void event(struct car *car, int event, int);
  bool key_cycle;
  int control_pin;
  int illumi_pin;
  int park_pin;
  int camera_pin;
  struct radio_command *last_command;
  long last_command_ms;
  long command_sequence;
  DueTimer *timer;
  bool busy;
  int cur_bit;
  uint8_t data[49];
  int commands;
  struct radio_command command[MAX_RADIO_COMMANDS];
} radio_t;

struct radio my_radio;

#define EVENT_HIT(x, mask)  ((param & (mask)) == (x))
#define EVENT_MISS(x, mask) ((param & (mask)) != (x))

#define DECLARE_RADIO_COMMAND(_name, _radio, _function, _match_fn, _fn) \
  do { \
    struct radio *radio = _radio; \
    struct radio_command *command = &radio->command[radio->commands]; \
    assert(radio->commands < MAX_RADIO_COMMANDS); \
    command->name = _name; \
    command->function = _function; \
    command->match_fn = [](int param)->bool { return _match_fn; }; \
    command->fn = [](struct radio_command *command) { _fn; }; \
    command->radio = radio; \
    radio->commands++; \
  } while (0)

void radio_set_keypress_delay(struct radio *radio)
{
  radio->last_command = NULL;
  radio->command_sequence = 0;
  radio->last_command_ms = 0;  
}

#define N 100

void radio_send_bits(struct radio_command *command, const uint8_t bits[])
{
  struct radio *radio = command->radio;

  if (radio->busy)
    return;

  if (command == radio->last_command)
    radio->command_sequence++;
  else
    radio_set_keypress_delay(radio);

// delay after first event in a sequence
  if ((millis() - radio->last_command_ms < RADIO_EVENT_FIRST_REPEAT_DELAY_MS))
    return;

  radio->last_command = command;
  if (radio->command_sequence == 0)
    radio->last_command_ms = millis();
  
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
//  DECLARE_RADIO_COMMAND("next screen",    &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b0110, 0b1111), radio_arm_next_screen());
  DECLARE_RADIO_COMMAND("swc combo",      &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b0110, 0b1111), radio_swc_combo());
  DECLARE_RADIO_COMMAND("toggle CAN-bus", &my_radio, RADIO_EVENT_SWC, EVENT_HIT(0b1001, 0b1111), radio_killswitch());
  DECLARE_RADIO_COMMAND("rearm keypress delay",  &my_radio, RADIO_EVENT_SWC,  EVENT_HIT(0b1111, 0b1111), radio_set_keypress_delay(command->radio));
 
  // Volvo: 1 - day, 0 - night
  // Kenwood: +12v - dimmer on
  // ULN2003A or NPN transistor reverse value (0 - +12v, 1 - 0v)

  DECLARE_RADIO_COMMAND("dimmer on",  &my_radio, RADIO_EVENT_ILLUMI,  EVENT_HIT (0b1, 0b1), digitalWrite(command->radio->illumi_pin, LOW));
  DECLARE_RADIO_COMMAND("dimmer off", &my_radio, RADIO_EVENT_ILLUMI,  EVENT_HIT (0b0, 0b1), digitalWrite(command->radio->illumi_pin, HIGH));

  DECLARE_RADIO_COMMAND("P gear on",  &my_radio, RADIO_EVENT_GEARBOX, EVENT_HIT (0b00, 0b11), digitalWrite(command->radio->park_pin, HIGH));
  DECLARE_RADIO_COMMAND("P gear off", &my_radio, RADIO_EVENT_GEARBOX, EVENT_MISS(0b00, 0b11), digitalWrite(command->radio->park_pin, LOW));

  DECLARE_RADIO_COMMAND("R gear on",  &my_radio, RADIO_EVENT_GEARBOX, EVENT_HIT (0b01, 0b11), digitalWrite(command->radio->camera_pin, LOW));
  DECLARE_RADIO_COMMAND("R gear off", &my_radio, RADIO_EVENT_GEARBOX, EVENT_MISS(0b01, 0b11), digitalWrite(command->radio->camera_pin, HIGH));
}

static int screen_arm_busy = false;
static DueTimer *screen_arm_timer = &Timer7;

void radio_arm_next_screen()
{
  if (screen_arm_busy)
    return;

  screen_arm_busy = true;
  screen_arm_timer->attachInterrupt(radio_unarm_delay).start(1000000);
  genie_next_screen();
}

void radio_swc_combo()
{
  if (screen_arm_busy)
    return;

  if (!my_display.enabled) { // enable it
    set_can_poll(&my_car, !my_car.can_poll);
    my_display.enabled = my_car.can_poll;
    set_widget(&my_display, "Can poll", my_car.can_poll);
  }

  if (my_radio.key_cycle)
    radio_arm_next_screen();
  else
    genie_set_screen(my_display.max_screen);
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

  set_can_poll(&my_car, !my_car.can_poll);
  my_display.enabled = my_car.can_poll; // turn off display if can't poll
  set_widget(&my_display, "Can poll", my_car.can_poll);
}

void radio_killswitch()
{
  loop_now_update = false; // watchdog will kill us shortly
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

void cem_ambient_light_cb(struct sensor *sensor)
{
  int level = get_sensor_value(sensor, 1);
  radio_event(&my_radio, RADIO_EVENT_ILLUMI, level != 0xf);
  my_display.light_level = level;
}

void tcm_gearbox_position_cb(struct sensor *sensor)
{
  radio_event(&my_radio, RADIO_EVENT_GEARBOX, get_sensor_value(sensor, 1));
}

void event_genie_touch(struct genie_widget *widget)
{
  genie_set_screen(10); // menu
}

void ccm_switch_status_cb(struct sensor *sensor)
{
  static bool last_status = 0, once = false;
  static long last_pressed_time = 0;
  bool status = get_sensor_value(sensor, 1);

  if (status && !last_status) {         /* press */
    last_pressed_time = millis();
  } else if (!status && last_status) {  /* release */
    if (!once) {
      if (millis() - last_pressed_time < 1000) {
        radio_swc_combo(); //genie_next_screen();
      } else {
        genie_previous_screen();
      }
    }
    once = false;
  } else if (status && last_status) {   /* press and hold */
    if (!once && (millis() - last_pressed_time > 3000)) {
      once = true;
      set_can_poll(&my_car, !my_car.can_poll);
      my_display.enabled = my_car.can_poll;
    }
  }
  last_status = status;
  sensor->acked = true;
}

void display_event_callback(void)
{
  genieFrame Event;
  struct genie_display *display = &my_display;

  display->genie.DequeueEvent(&Event);

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT) {
    for (int i = 0; i < display->widget_count; i++) {
      if ((display->widget[i].object_type  == Event.reportObject.object) &&
          (display->widget[i].object_index == Event.reportObject.index)) {
        struct genie_widget *widget = &display->widget[i];
        widget->current_value = display->genie.GetEventData(&Event);

        //if (debug_print)
          SerialEx.printf("input %s data %d\n", widget->name, widget->current_value);
        if (widget->cb_fn)
          widget->cb_fn(widget);
      }
    }
  }
}

void rse_left_display_en(bool en)
{
  static bool inited = false;
  static bool last_state = false;
  if (inited && (en == last_state))
    return;
  last_state = en;
  if (debug_print)
    SerialEx.printf("left display %d\n", en);
  digitalWrite(RSE_LEFT_DISPLAY_EN_PIN, !en);
  eeprom_store(EEPROM_RSE_LEFT_EN, en);
}

void rse_right_display_en(bool en)
{
  static bool inited = false;
  static bool last_state = false;
  if (inited && (en == last_state))
    return;
  last_state = en;
  if (debug_print)
    SerialEx.printf("right display %d\n", en);
  digitalWrite(RSE_RIGHT_DISPLAY_EN_PIN, !en);
  eeprom_store(EEPROM_RSE_RIGHT_EN, en);
}

void event_gps_navigation(struct genie_widget *widget)
{
  SerialEx.printf("event GPS nav\n");
  rti_en(widget->current_value);
}

void event_left_display(struct genie_widget *widget)
{
  SerialEx.printf("event left display\n");
  rse_left_display_en(widget->current_value);
}

void event_right_display(struct genie_widget *widget)
{
  SerialEx.printf("event right display\n");
  rse_right_display_en(widget->current_value);
}

void event_sri_reset(struct genie_widget *widget)
{
  CAN_FRAME out;

  if (debug_print)
    SerialEx.printf("SRI reset\n");

  memset(out.data.bytes, 0, 8);
  out.id = 0x0ffffe;
  out.extended = true;
  out.priority = 4;
  out.length = 8;

  out.data.byte[0] = 0xcb;
  out.data.byte[1] = 0x51;
  out.data.byte[2] = 0xb2;
  out.data.byte[3] = 0x01;

  //  print_frame("OUT", &out);
  CAN_HS.sendFrame(out);
}

void event_transmission_adaptation(struct genie_widget *widget)
{
  CAN_FRAME out;

  if (debug_print)
    SerialEx.printf("Transmission adaptation\n");

  memset(out.data.bytes, 0, 8);
  out.id = 0x0ffffe;
  out.extended = true;
  out.priority = 4;
  out.length = 8;

  out.data.byte[0] = 0xcb;
  out.data.byte[1] = 0x6e;
  out.data.byte[2] = 0xb2;
  out.data.byte[3] = 0x50;

  //  print_frame("OUT", &out);
  CAN_HS.sendFrame(out);
}

void event_can_poll(struct genie_widget *widget)
{
  SerialEx.printf("event CAN poll\n");

  if (widget->current_value != widget->display->car->can_poll) {
    set_can_poll(widget->display->car, !!widget->current_value);
  }
}

void event_key_cycle(struct genie_widget *widget)
{
  SerialEx.printf("event key cycle\n");
  if (widget->current_value != my_radio.key_cycle) {
    my_radio.key_cycle = !!widget->current_value;
    eeprom_store(EEPROM_KEY_CYCLE, my_radio.key_cycle);
  }
}

void event_goto_screen(struct genie_widget *widget)
{
  SerialEx.printf("event goto screen %d\n", widget->object_index);
  genie_set_screen(widget->object_index);
}

extern "C" void _watchdogEnable (void) {}
extern "C" void _watchdogDisable (void) {}

void watchdogSetup (void) __attribute__ ((weak, alias("_watchdogEnable")));
void watchdogShutoff (void) __attribute__ ((weak, alias("_watchdogDisable")));

int eeprom_ready = 0;

void setup_eeprom(void)
{
  unsigned char magic_0, magic_1;

  eeprom_init(7, 12, 9, 8);
  eeprom_ewen();
  for (int i = 0; i < 2; i++) {
    magic_0 = eeprom_read(0);
    magic_1 = eeprom_read(1);
    SerialEx.printf("[%d] EEPROM magic_0 = 0x%02x\n", i, magic_0);
    SerialEx.printf("[%d] EEPROM magic_1 = 0x%02x\n", i, magic_1);
    if ((magic_0 == EEPROM_MAGIC_0) &&
        (magic_1 == EEPROM_MAGIC_1)) {
        eeprom_ready = 1;
        break;
    } else { // try to init EEPROM
       eeprom_write(0, EEPROM_MAGIC_0);
       eeprom_write(1, EEPROM_MAGIC_1);
    }
  }
  if (eeprom_ready)
    SerialEx.printf("EEPROM OK\n");
  else
    SerialEx.printf("EEPROM was not found\n");
}

unsigned char eeprom_load(int address, unsigned char def)
{
  unsigned char value;

  if (!eeprom_ready)
    return def;
  value = eeprom_read(address);
  if (debug_print)
    SerialEx.printf("load from EEPROM at address %d, got value %d\n", address, value);
  return value;
}

void eeprom_store(int address, unsigned char value)
{
  if (!eeprom_ready)
    return;
  if (debug_print)
    SerialEx.printf("store to EEPROM at address %d value %d\n", address, value);
  eeprom_write(address, value);
}

void set_can_poll(struct car *car, bool en)
{
  if (car->can_poll != en)
    eeprom_store(EEPROM_CAN_POLL, en);

  car->can_poll = en;
  if (en) /* that works only once after CPU reset */
    watchdogSetup();
  else
   watchdogShutoff();
}

void setup_eeprom(genie_display *display)
{
  bool en;

  current_screen = eeprom_load(EEPROM_CURRENT_SCREEN, 0) % display->max_screen;

  en = eeprom_load(EEPROM_CAN_POLL, 1);
  set_widget(display, "Can poll", en);
  set_can_poll(display->car, en);

  en = eeprom_load(EEPROM_RSE_LEFT_EN, 0);
  set_widget(display, "Left display", en);
  rse_left_display_en(en);

  en = eeprom_load(EEPROM_RSE_RIGHT_EN, 0);
  set_widget(&my_display, "Right display", en);
  rse_right_display_en(en);

  en = eeprom_load(EEPROM_RTI_EN, 0);
  set_widget(&my_display, "GPS navigation", en);
  rti_en(en);

  en = eeprom_load(EEPROM_KEY_CYCLE, 0);
  set_widget(&my_display, "Key cycle", en);
  my_radio.key_cycle = en;

}

///////////////////// RTI ////////////////////////////////
// levels are inverted

static DueTimer *rti_serial_timer = &Timer5;

static unsigned char rti_bytes[] = { 0x40, 0x40, 0x83 };
int rti_byte = 0;
int rti_bit = 0;
bool rti_enabled = false;

void rti_en(bool en)
{
  static bool last_state = false;
  if (en == last_state)
    return;
  last_state = en;

  eeprom_store(EEPROM_RTI_EN, en);

  if (en) {
    rti_byte = 0;
    rti_enabled = true;
    pinMode(RTI_PIN, OUTPUT);
    digitalWrite(RTI_PIN, !HIGH);
    rti_serial_timer->attachInterrupt(rti_isr_send_start_bit).start(1000); // 1 ms
  } else {
    rti_enabled = false;
    rti_serial_timer->stop();
  }
}

void rti_isr_send_start_bit()
{
  if (!rti_enabled)
    return;

  rti_bit = 0;
  digitalWrite(RTI_PIN, !LOW); // start bit
  rti_serial_timer->attachInterrupt(rti_isr_send_bit).setFrequency(2400).start(); // 2400 baud
}

void rti_isr_send_bit()
{
  unsigned char byte = rti_bytes[rti_byte];
  bool bit = byte & (1 << rti_bit);

  if (!rti_enabled)
    return;

  digitalWrite(RTI_PIN, !bit);
  rti_bit++;
  if (rti_bit >= 8)
    rti_serial_timer->attachInterrupt(rti_isr_send_stop_bit).setFrequency(2400).start();
}

void rti_isr_send_stop_bit()
{
  if (!rti_enabled)
    return;

  rti_byte = (rti_byte + 1) % (sizeof(rti_bytes));
  digitalWrite(RTI_PIN, !HIGH);
  rti_serial_timer->attachInterrupt(rti_isr_send_start_bit).start(100000); // 100 ms between bytes
}

//////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  SerialEx.printf("start\n");
  SerialEx.printf("watchdog timeout %d ms\n", WDT_TIMEOUT);
  setup_car(&my_car);
  setup_eeprom();
  setup_radio(&my_radio);
#ifndef NO_CAN
  setup_canbus(&my_car);
#endif
  setup_genie_display(&my_display, &my_car);
  setup_eeprom(&my_display);
}

void loop()
{
  while (1) {
#ifdef NO_CAN
    watchdog_reset();
#endif
    if (loop_now_update)
      loop_now = millis();
#ifdef NO_CAN_CALLBACK
    check_cans(&my_car);
#endif
    query_all_sensors(&my_car);
    refresh_display(&my_display, current_screen);
  }
}
