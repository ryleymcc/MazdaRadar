#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <globals.h>
#include <helpers.h>
#include <defs.h>
IntervalTimer canTimer;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> mazda;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> radar;
CAN_message_t msg;

uint64_t frame = 0;

void setup(void) {
  mazda.begin();
  mazda.setBaudRate(500000);
  radar.begin();
  radar.setBaudRate(500000);
  canTimer.begin(tick, 500); //5ms
#ifdef DEBUG 
  Serial.begin(9600);
  Serial.println("start");
#endif
}

void loop() {
  begin = true;
  if ( mazda.read(msg) ) {
    // GET OP COMMAND
    switch (msg.id) {
      case 0x1E0: {
        can_tick = 0; //reset timer
        accel_request = get_accel_request(msg);
        //op_acc_enabled = get_op_acc_enabled(msg);
#ifdef DEBUG 
        if (frame % 100 == 0) {
          Serial.print(msg.id, HEX);
          Serial.print(": ");
          // Print the message .buf array
          for (int i = 0; i < msg.len; i++) {
            Serial.print(msg.buf[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        }
#endif
        break;
      }
      case 0x228: {
        gear = (msg.buf[0] & 4);
        //Serial.println(msg.buf[0], HEX);
        break;
      }
      case 0x9D: {
        cancel = (msg.buf[0] & 1) == 1;
        break;
      }
    }
    radar.write(msg);

  }else if ( radar.read(msg) ) {
    frame++;
    if (begin) { // Let the system settle. May not be needed
      // Handle the messages
      switch (msg.id) {
        case 0x21b: {
          // check if acc_active is true

          crz_ended = ((uint64_t)msg.buf[4] & 0x10) << 24;
          acc_allowed = ((uint64_t)msg.buf[4] & 0x4) << 24;
          if (acc_active) {
            if (cancel) {
              acc_active = ((uint64_t)msg.buf[4] & 0x2) << 24;
            }else{
              acc_active = ((uint64_t)0xFF & 0x2) << 24;
            }
          }
          else{
            acc_active = ((uint64_t)msg.buf[4] & 0x2) << 24;
          }
          uint64_t mystery_bit = ((uint64_t)msg.buf[5] & 0x80) << 16;
          counter = ((uint64_t)msg.buf[6] & 0x0F) << 8;
          // Read the acceleration command
          radar_acc_command = get_radar_acc_command(msg);
          crz_ended = ((uint64_t)0x0 & 0x10) << 24;
          if (gear) {
            acc_allowed = ((uint64_t)0xFF & 0x4) << 24;
            //Serial.println("Gear");
          }
          if (acc_active) {
            #ifdef DEBUG 
            if (frame % 10 == 0) {
              Serial.print("radar_acc_command: ");
              Serial.println(radar_acc_command);
            }
            #endif
            if (can_tick < 40) { // if OP command is old
                if (accel_request < (radar_acc_command >> 29)) {
                  radar_acc_command = (((uint64_t)applyAccelLimits()) << 29);
                }

                //radar_acc_command = (((uint64_t)4096 - 1000) << 29);
            }
            #ifdef DEBUG           
            if (frame % 10 == 0) {
              Serial.print("radar_acc_command: ");
              Serial.println(radar_acc_command >> 29);
            }
            #endif          
          }

          uint64_t data = STATIC_DATA_21B | counter | radar_acc_command | acc_active | acc_allowed | crz_ended | mystery_bit;
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          data = __builtin_bswap64 (data);
          data |= checksum(msg.buf);
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          break;
        }
        case 0x21c: {
          break;
        }

        case 0x361: {
          // Add all the signals together
          uint64_t data = STATIC_DATA_361 | get_inverse_speed(msg) | get_counter(msg);
          // Change endinaness
          data = __builtin_bswap64 (data);
          // Copy the data to the msg.buf
          memcpy(msg.buf, &data, 8);
          break;
        }
        
        case 0x362: {
          uint64_t data = STATIC_DATA_362 | get_steering_angle(msg) | get_counter(msg);       
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);                             
          break;
        }

        case 0x363: {
          uint64_t data = STATIC_DATA_363 | get_counter(msg);
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          break;
        }

        case 0x364: {
          uint64_t data = STATIC_DATA_364 | get_counter(msg);
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          break;
        }

        case 0x365: {
          uint64_t data = STATIC_DATA_365 | get_counter(msg);
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          break;
        }

        case 0x366:
          {
          uint64_t data = STATIC_DATA_366 | get_counter(msg);
          data = __builtin_bswap64 (data);
          memcpy(msg.buf, &data, 8);
          break;
        }

        default:
          break;
      } // end switch
    } // end of begin
    mazda.write(msg);
  } 
}