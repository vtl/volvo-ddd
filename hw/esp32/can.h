#include <mcp_can.h>

#define CAN_BPS_500K CAN_500KBPS
#define CAN_BPS_125K CAN_125KBPS

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

typedef struct {
  MCP_CAN can;
  uint8_t cs_pin;
  uint8_t int_pin;
  byte filt_idx = 0;
  void(*cb)(CAN_FRAME *);
  bool begin(uint8_t rate);
  byte read(CAN_FRAME &in);
  void sendFrame(CAN_FRAME& out);
  void setRXFilter(uint32_t can_id, uint32_t mask, bool extended);
  void setGeneralCallback(void(*fn)(CAN_FRAME *));
  void isr(void);
} CANRaw;

CANRaw CAN_HS;
CANRaw CAN_LS;

bool CANRaw::begin(uint8_t rate)
{
  dprintf("CS %d rate %d\n", cs_pin, rate);
  can.init_CS(cs_pin);
  return can.begin(rate, MCP_8MHz) == MCP2515_OK;
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
  CAN_FRAME in;

  dprintf("ISR\n");

  while (can.readMsgBuf(&in.length, in.data.bytes)) {
    if (cb)
      cb(&in);
  }
}

void CANRaw::setGeneralCallback(void(*fn)(CAN_FRAME *))
{
  cb = fn;
  attachInterrupt(digitalPinToInterrupt(int_pin), (this == &CAN_HS) ? can_hs_isr : can_ls_isr, FALLING);
}

void CANRaw::setRXFilter(uint32_t can_id, uint32_t mask, bool extended)
{
  dprintf("%s filt %d, can_id 0x%x, mask 0x%x, extended %d\n", __func__, filt_idx, can_id, mask, extended);
// FIXME don't set masks every time
  can.init_Mask(0, extended, mask);
  can.init_Mask(1, extended, mask);
  can.init_Filt(filt_idx++, extended, can_id);
}

void CANRaw::sendFrame(CAN_FRAME& out)
{
  int ret = can.sendMsgBuf(out.id, out.extended, 8, out.data.bytes);
  if (ret != CAN_OK)
    dprintf("sendFrame error\n");
}

byte can_read(struct MCP_CAN *can, CAN_FRAME *in)
{
  byte ret;

  ret = can->checkReceive();
  if (ret == CAN_MSGAVAIL)
    ret = can->readMsgBufID( can->readRxTxStatus(), (volatile unsigned long *)&in->id, &in->extended, &in->rtr, &in->length, in->data.bytes);
  return ret;
}

byte CANRaw::read(CAN_FRAME &in)
{
  return can_read(&can, &in);
}
