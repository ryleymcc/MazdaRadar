#include <defs.h>

void tick(void) {
  can_tick++;
  rate_tmr = true;
}

static int clamp(int d, int min, int max) {
  const int t = d < min ? min : d;
  return t > max ? max :t;
}

static uint16_t applyAccelLimits(void)
{
  static uint16_t merged_accel;
  uint16_t apply_accel;
  uint16_t apply_accel_request;
  apply_accel_request = (uint16_t)clamp((int)accel_request, 4095 - ACCEL_REQ_MAX, 4095 + ACCEL_REQ_MAX);
  apply_accel = merged_accel;
  if (merged_accel > 4095){    //  accel is now positive    
    apply_accel = (uint16_t)(clamp(apply_accel_request, max(merged_accel - ACCEL_MAX_DOWN, -ACCEL_MAX_UP), \
                                                            (merged_accel + ACCEL_MAX_UP)) );
  }else{
    apply_accel = (uint16_t)(clamp(apply_accel_request, merged_accel - ACCEL_MAX_UP, \
                                    min((int)(merged_accel + ACCEL_MAX_DOWN), (int)(4095U + ACCEL_MAX_UP))) );
  }
  merged_accel = apply_accel;
  if (!acc_active){
    merged_accel = 4095;     //applied accel is always 0
    apply_accel = 4095;
  }
  return(apply_accel);
}

uint64_t get_data (CAN_message_t msg) {
  uint64_t data = 0;
  memcpy(&data, msg.buf, sizeof(data));
  return data;
}

uint16_t checksum(uint8_t *data) {
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++) {
    sum += data[i];
  }
  sum &= 0xFF;
  sum = ~sum;
  return sum;
}

// given the size of the signal, return a mask of the signal
uint64_t gen_mask(uint8_t *msb, uint8_t *lenth) {
  uint64_t mask;
  int sizeInBits = sizeof(mask) * 8; // BITS_PER_BYTE = 8;
  mask = (*lenth >= sizeInBits ? -1 : (1 <<  *lenth) - 1);
  //make a pair of the mask and the shift and return it
  return (mask);
}

// given the signals msb positions from the dbc file, update the array.
void gen_shift(uint8_t *msb) {
  for (uint8_t i = 0; i < sizeof(msb) / sizeof(msb[0]); i++) {
    uint8_t tmp = 64 - (((msb[i]/8)*8) + (7 - (msb[i] % 8)));
    Serial.print("tmp: ");
    Serial.println(tmp);
    msb[i] = tmp;
  }
}

uint8_t get_counter(CAN_message_t msg) {
  return (msg.buf[msg.len - 1]) & 0x0F;
}

uint64_t get_steering_angle(CAN_message_t msg) {
  uint64_t data;
  // Get the steering angle 0x00000000000FFF00
  data = ((msg.buf[5] & 0x0F) << 16) | ((msg.buf[6] & 0xFC) << 8);
  return data;
}

uint64_t get_inverse_speed(CAN_message_t msg) {
  uint64_t data;
  data = ((msg.buf[5] & 0x0F) << 16) | (msg.buf[6] << 8); 
  return data;
}

uint64_t get_distance_lead(CAN_message_t msg) {
  uint64_t data;
  data = (uint64_t)msg.buf[0] << 56 | (uint64_t)msg.buf[1] << 48 | (uint64_t)msg.buf[2] << 40;
  return data;
}

uint64_t get_relative_vel(CAN_message_t msg) {
  uint64_t data;
  data = (uint64_t)msg.buf[3] << 32 | ( ((uint64_t)msg.buf[4] & 0xE0) << 24);
  return data;
}

uint64_t get_radar_acc_command(CAN_message_t msg) {
  uint64_t data;
  data = ( ((uint64_t)msg.buf[2] & 0x03) << 40) | ((uint64_t)msg.buf[3] << 32) | ( ((uint64_t)msg.buf[4] & 0xC0) << 24);
  return data;
}
// acc_allowed
uint64_t get_acc_set_allowed(CAN_message_t msg) {
  uint64_t data;
  data = ( ((uint64_t)msg.buf[4] & 0x04) << 32);
  return data;
}

uint16_t get_accel_request(CAN_message_t msg) {
  uint64_t data;
  data = ((uint64_t)msg.buf[0] << 5) | msg.buf[1] >> 3;
  return data;
}