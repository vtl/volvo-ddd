#include <FlexCAN_T4.h>

#define CAN_BPS_500K 500000
#define CAN_BPS_125K 125000

typedef union {
  uint8_t bytes[8];
  uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct
{
  uint32_t id;            // EID if ide set, SID otherwise
  uint32_t fid;           // family ID
  uint8_t rtr;            // Remote Transmission Request
  uint8_t priority;       // Priority but only important for TX frames and then only for special uses.
  uint8_t extended;       // Extended ID flag
  uint8_t length;         // Number of data bytes
  BytesUnion data;        // 64 bits - lots of ways to access it.
} CAN_FRAME;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_ls;

typedef enum {
  CAN_BUS_HS,       /* high-speed bus */
  CAN_BUS_LS        /* low-speed bus */
} can_bus_id_t;

void can_hs_isr(void);
void can_ls_isr(void);

void can_hs_event (const CAN_message_t &msg)
{
  can_hs_isr();
}

void can_ls_event (const CAN_message_t &msg)
{
  can_ls_isr();
}

void can_ls_init(int baud)
{
  can_ls.begin();
  can_ls.setBaudRate(baud);
  can_ls.enableFIFO();
  can_ls.enableFIFOInterrupt();
  can_ls.setFIFOFilter(ACCEPT_ALL);
  can_ls.onReceive(can_ls_event);
  printf("CAN low-speed init done.\n");
}

void can_hs_init(int baud)
{
  can_hs.begin();
  can_hs.setBaudRate(baud);
  can_hs.enableFIFO();
  can_hs.enableFIFOInterrupt();
  can_hs.setFIFOFilter(ACCEPT_ALL);
  can_hs.onReceive(can_hs_event);
  printf("CAN high-speed init done.\n");
}

typedef struct CANRaw {
  can_bus_id_t bus;
  uint8_t cs_pin;
  uint8_t int_pin;
  int clk;
  byte filt_idx = 0;
  void(*cb)(CAN_FRAME *);
  bool begin(int rate);
  byte read(CAN_FRAME &in);
  void sendFrame(CAN_FRAME& out);
  void setRXFilter(uint32_t can_id, uint32_t mask, bool extended);
  void setGeneralCallback(void(*fn)(CAN_FRAME *));
  void isr(void);
  const char *strerror(int);
  CANRaw(can_bus_id_t _bus) { bus = _bus; }
} CANRaw;

CANRaw CAN_HS(CAN_BUS_HS);
CANRaw CAN_LS(CAN_BUS_LS);

bool CANRaw::begin(int rate)
{
  dprintf("bus %d, rate %d\n", bus, rate);
  if (bus == CAN_BUS_HS)
	  can_hs_init(rate);
  else
	  can_ls_init(rate);

  return true;
}

void can_hs_isr(void)
{
  CAN_HS.isr();
}

void can_ls_isr(void)
{
  CAN_LS.isr();
}

void CANRaw::isr(void)
{
#if 0
  CAN_FRAME in;

  dprintf("ISR\n");

  while (can.readMsgBuf(&in.length, in.data.bytes)) {
    if (cb)
      cb(&in);
  }
#endif
}

void CANRaw::setGeneralCallback(void(*fn)(CAN_FRAME *))
{
  cb = fn;
//  attachInterrupt(digitalPinToInterrupt(int_pin), (this == &CAN_HS) ? can_hs_isr : can_ls_isr, FALLING);
}

void CANRaw::setRXFilter(uint32_t can_id, uint32_t mask, bool extended)
{
  dprintf("%s filt %d, can_id 0x%lx, mask 0x%lx, extended %d\n", __func__, filt_idx, can_id, mask, extended);
// FIXME don't set masks every time
#if 0
  can.init_Mask(0, extended, mask);
  can.init_Mask(1, extended, mask);
  can.init_Filt(filt_idx++, extended, can_id);
#endif
}

void CANRaw::sendFrame(CAN_FRAME& out)
{
  int ret;
  CAN_message_t msg;

  msg.id = out.id;
  msg.flags.extended = out.extended;
  msg.len = out.length;
  memcpy(msg.buf, out.data.bytes, msg.len);

  if (bus == CAN_BUS_HS)
	  ret = can_hs.write(msg);
  else
	  ret = can_ls.write(msg);

  if (ret != 0)
    dprintf("sendFrame error: %s (%d)\n", CANRaw::strerror(ret), ret);
}

byte can_read(can_bus_id_t bus, CAN_FRAME *in)
{
  byte ret;
  CAN_message_t msg;

  if (bus == CAN_BUS_HS)
	  ret = can_hs.read(msg);
  else
	  ret = can_ls.read(msg);

  if (ret) {
	  in->id = msg.id;
	  in->length = msg.len;
	  in->extended = msg.flags.extended;
	  memcpy(in->data.bytes, msg.buf, in->length);
  }

  return ret;
}

byte CANRaw::read(CAN_FRAME &in)
{
  return can_read(bus, &in);
}

#if 0
static struct _can_errors {
  int code;
  const char *msg;
} can_errors[] = {
  { CAN_OK, "CAN_OK" },
  { CAN_FAILINIT, "CAN_FAILINIT" },
  { CAN_FAILTX, "CAN_FAILTX" },
  { CAN_MSGAVAIL, "CAN_MSGAVAIL" },
  { CAN_NOMSG, "CAN_NOMSG" },
  { CAN_CTRLERROR, "CAN_CTRLERROR" },
  { CAN_GETTXBFTIMEOUT, "CAN_GETTXBFTIMEOUT" },
  { CAN_SENDMSGTIMEOUT, "CAN_SENDMSGTIMEOUT" },
  { CAN_FAIL, "CAN_FAIL" }};
#endif

const char *CANRaw::strerror(int code)
{
#if 0
  for(unsigned i = 0; i < sizeof(can_errors) / sizeof(struct _can_errors); i++)
    if (code == can_errors[i].code)
      return can_errors[i].msg;
#endif
  return "?";
}

#define CAN_OK 0
#define CAN_NOMSG 1

void can_setup()
{
}
