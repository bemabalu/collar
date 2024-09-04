#include "CollarRxType1.h"

#define TYPE1_START_PULSE_LEN_US 2200
#define TYPE1_START_PULSE_TOLLERANCE 100
#define TYPE1_PULSE_LEN_US 1000
#define TYPE1_PULSE_TOLLERANCE 100
#define TYPE1_HIGH_THERESHOLD 500

CollarRxType1::CollarRxType1(uint8_t rx_pin, msg_cb_t cb, void *userdata, uint16_t id) : CollarRx { rx_pin, cb, userdata, id }
{

}

CollarRxType1::CollarRxType1(uint8_t rx_pin, msg_cb_t cb, void *userdata) : CollarRx { rx_pin, cb, userdata } 
{

}

void CollarRxType1::buffer_to_collar_message(const uint8_t buffer[5], struct collar_message *msg)
{
  // wipe old message
  memset(msg, 0, sizeof(struct collar_message));

  // bytes 0&1 = ID
  msg->id=((buffer[0]<<8)|buffer[1]);

  // byte 2 = mode & channel
  msg->mode     = (collar_mode)(buffer[2] & 0x0F);
  msg->channel  = (collar_channel)((buffer[2] & 0xF0) >> 4);

  // byte 3 = power
  msg->power    =  buffer[3];

  // byte 4 = checksum
}

bool CollarRxType1::is_message_valid(const uint8_t buffer[5])
{
  // if we're filtering by ID, check it matches
  if (_use_id && (memcmp(buffer, &_id, 2)))
    return false;

  // calculate checksum
  uint8_t chk=0;
  for (uint8_t i=0; i < 4; i++)
    chk += buffer[i];

  // validate checksum is correct
  if (chk != buffer[4])
    return false;

  return true;
}

void CollarRxType1::isr()
{
  static unsigned long rx_micros =0;
  static unsigned int high_pulse_len =0;
  static unsigned int low_pulse_len=0;
  static uint8_t buffer[5];        // expecting to receive 5 bytes
  static uint8_t byte_position = -1; // keep track of current byte being received, , if -1 no start received
  static uint8_t bit_position = 0; // keep track of expected next bit postion in byte

  if(digitalRead(_rx_pin) == 0)
  { //falling edge
    high_pulse_len = micros() - rx_micros;
    rx_micros = micros(); //start measurement of pulse length for low state
    return;
  } 
  else 
  {  //rising edge
    low_pulse_len = micros() - rx_micros;
    rx_micros = micros(); //start measurement of pulse length for high state
  
    // look for tranmission start state (rising edge->rising edge of ~2.2ms)
    int last_pulse_len = high_pulse_len+low_pulse_len;
    if ((last_pulse_len > TYPE1_START_PULSE_LEN_US-TYPE1_START_PULSE_TOLLERANCE) &&
       (last_pulse_len < TYPE1_START_PULSE_LEN_US+TYPE1_START_PULSE_TOLLERANCE))
    {
        //pulse_count = 0;
        byte_position = 0;
        bit_position = 0;
        memset(buffer, 0, sizeof(buffer));
        return;
    }
    else
    { 
      if (byte_position>=0){ //we are receiving data   
        if ((last_pulse_len > TYPE1_PULSE_LEN_US-TYPE1_PULSE_TOLLERANCE) &&
            (last_pulse_len < TYPE1_PULSE_LEN_US+TYPE1_PULSE_TOLLERANCE))
        { //seems we got a valid bit     
          uint8_t val=(high_pulse_len>TYPE1_HIGH_THERESHOLD); //if high longer than 500ms we got a 1 else we got a 0  

          if (byte_position < sizeof(buffer))
          {
            
            if (val)
              buffer[byte_position] |= (1 << (7-bit_position));

            if (++bit_position >= 8)
            {
              bit_position = 0;
              byte_position++;
            }

            if (byte_position >=  sizeof(buffer))
            {
              if (is_message_valid(buffer))
              {
                buffer_to_collar_message(buffer, &_rx_msg);
                _cb(&_rx_msg, _userdata);
                byte_position=-1; //done, wait for new start
                return;
              }
            }
          }
        }
        else
        {
          //transmission error, wait for new start
          byte_position=-1;
          return;
        }
      }
    }
  }
}
