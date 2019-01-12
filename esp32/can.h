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
  void(*cb)(CAN_FRAME *);
  bool begin(uint8_t rate);
  void sendFrame(CAN_FRAME out);
  void setRXFilter(int can_id, uint32_t mask, bool extended);
  void setGeneralCallback(void(*fn)(CAN_FRAME *));
} CANRaw;

CANRaw CAN_HS;
CANRaw CAN_LS;

bool CANRaw::begin(uint8_t rate)
{
  can.init_CS(cs_pin);
  return can.begin(rate, MCP_8MHz);
}

void isr(void)
{
  uint8_t len;
  CAN_FRAME in;
#if 0
  while (can.readMsgBuf(&len, in.data.bytes)) {
    if (cb)
      cb(&in);
  }
#endif
}

void CANRaw::setGeneralCallback(void(*fn)(CAN_FRAME *))
{
  cb = fn;
  attachInterrupt(digitalPinToInterrupt(int_pin), isr, FALLING);
}

void CANRaw::setRXFilter(int can_id, uint32_t mask, bool extended)
{
  printf("%s stub can_id 0x%x, mask 0x%x, extended %d\n", __func__, can_id, mask, extended);
}

void CANRaw::sendFrame(CAN_FRAME out)
{
//  can->
}

byte can_read(struct MCP_CAN *can, CAN_FRAME *in)
{
  return can->readMsgBufID(can->readRxTxStatus(), (volatile unsigned long *)&in->id, &in->extended, &in->rtr, &in->length, in->data.bytes);
}
