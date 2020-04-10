/* free_memory() => 447 (before transition to state struct)
 *  454 => using Events ring buffer and state struct for switch
 * need 432 bytes for HSV[144] buffer to accomodate high density 1m strips
 */
#include <Arduino.h>
#include <avr/eeprom.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdbool.h>

#define DEBUG

// used with DEBUG
#include "ATtinySerialOut.h"  // https://github.com/ArminJo/ATtinySerialOut (defaults to PB2)
#include "memoryFree.h"       // for memoryFree();

#define adc_disable() (ADCSRA &= ~(1<<ADEN))  // disable ADC (before power-off) as it uses ~320uA
#define acomp_disable() (ACSR |= _BV(ACD))    // disable analog comparator for power savings

// Pin assignments
#define PIN_ENCODER0   PB0
#define PIN_ENCODER1   PB1
#define PIN_PCB_LED    PB1
#define PIN_TINYSERIAL PB2
#define PIN_SWITCH     PB4

// Encoder & switch bitmasks to apply to PINB register
// TODO: move to dedicated header file
#define ENCODER_CLK (1 << PIN_ENCODER0)
#define ENCODER_DT (1 << PIN_ENCODER1)
#define ENCODER (ENCODER_CLK | ENCODER_DT)
#define ENCODER_LEFT ENCODER_CLK
#define ENCODER_LEFT_A ENCODER_DT
#define ENCODER_LEFT_B ENCODER_CLK
#define ENCODER_RIGHT ENCODER
#define ENCODER_RIGHT_A (ENCODER_CLK & ENCODER_DT)
#define ENCODER_RIGHT_B (ENCODER_CLK | ENCODER_DT)
#define SWITCH (1 << PIN_SWITCH)

// For tuning linear encoder acceleration, TODO: consider exponential or logarithmic
#define ENCODER_SCALE_WINDOW 80  // acceleration window in ms
#define ENCODER_SCALE_FACTOR 5   // max linear accelleration = window / factor 

// EEPROM map
const uint16_t * Eeprom_Startup_Mode = (uint16_t *) 0x00;
const uint16_t * Eeprom_Strip_Length = (uint16_t *) 0x04;
const uint8_t * Eeprom_M0_Off = (uint8_t *) 0x10;
const uint8_t * Eeprom_M0_Setup = (uint8_t *) 0x20;
const uint8_t * Eeprom_M0_Solid = (uint8_t *) 0x30;
const uint8_t * Eeprom_M0_Wheel = (uint8_t *) 0x40;
const uint8_t * Eeprom_M0_Breathe = (uint8_t *) 0x50;
const uint16_t * Eeprom_End = (uint16_t *) 0x60;

// Interrupt event queue
#define MAX_EVENT_COUNT 4  // how many events to queue, TODO: tune (reduce & backoff)

enum Event_Types {
  Encoder_Left,
  Encoder_Right,
  Switch_Down,
  Switch_Up,
};

const uint16_t event_queue_time_mask = 0b111111111111;

struct Event_Queue {
  uint16_t event : 4;
  uint16_t time : 12;
};

// FIFO ring buffer; push to end, pop from front
struct Events {  // sizeof() = 1 + 2 * MAX_EVENT_COUNT
  uint8_t start : 4;
  uint8_t length : 4;
  struct Event_Queue queue[MAX_EVENT_COUNT];
} events;

volatile uint8_t pinb_history;     // to determine which pin change event caused an interrupt
volatile uint8_t encoder_history;  // to store first phase of quadrature events

// last_* use the lower 15 bits to track 32 seconds of time; the top bit to determine state
// TODO: convert to bitmaps (?)
#define LAST_ACTIVE (uint16_t) (1 << 15)
uint16_t last_encoder_left = 0;
uint16_t last_encoder_right = 0;

#define SWITCH_PRESS_LONG_MS 750         // ms to count as a long switch event
#define SWITCH_PRESS_SUPER_LONG_MS 2000  // ms to count as a super long switch event
#define SWITCH_PRESS_MEGA_LONG_MS 10000  // ms to mega long (reset event)

// Right now, this is a masked byte; but this could (should) be a 2-4 bit counter 
enum Switch_States {
  Switch_Press_Short = 1 << 0,
  Switch_Press_Long = 1 << 1,
  Switch_Press_Super_Long = 1 << 2,
  Switch_Press_Mega_Long = 1 << 3,
};

const uint16_t state_last_time_mask = 0b111111111111111;

struct State {
  uint8_t switch_press_short : 1;
  uint8_t switch_press_long : 1;
  uint8_t switch_press_super_long : 1;
  uint8_t switch_press_mega_long : 1;
  uint16_t switch_last_active : 1;
  uint16_t switch_last_time : 15;
} state;

enum Mode_0 {
  M0_Off,
  M0_Setup,
  M0_Solid,
  M0_Wheel,
};

enum Mode_1_Setup {
  M1_Setup_Length = 0,
  M1_Setup_Reserved = 1 << 4,
};

enum Mode_1_Solid {
  M1_Solid_Brightness = 0,
  M1_Solid_Hue = 1 << 4,
  M1_Solid_Saturation = 1 << 5,
};
/*
enum Mode_1_Wheel {
  M1_Wheel_Brightness = 0, 
  M1_Wheel_Saturation = 1 << 4,
  M1_Wheel_Speed = 1 << 5,
};

enum Mode_1_Breathe {
  M1_Breathe_Brightness = 0, 
  M1_Breathe_Hue = 1 << 4,
  M1_Breathe_Saturation = 1 << 5,
  M1_Breathe_Speed = 1 << 6,
};
*/
uint16_t mode = 0;  // 

union mode_state {
  struct {
    uint16_t strip_length;
  } setup;
  struct {
    uint8_t brightness;    
    uint8_t hue;
    uint8_t saturation;
  } solid;
  struct {
    uint8_t brightness;
    uint8_t saturation;
    uint8_t speed;
  } wheel;
  struct {
    uint8_t brightness;
    uint8_t hue;
    uint8_t saturation;
    uint8_t speed;
  } breath;
} mode_state;


// encoder functions
inline uint8_t encoder_accel(uint16_t) __attribute__((always_inline));
// inline uint8_t encoder_accel_linear(uint16_t) __attribute__((always_inline));
// inline uint8_t encoder_accel_log(uint16_t) __attribute__((always_inline));
inline uint8_t encoder_rotate_left(uint16_t, uint8_t);
inline uint8_t encoder_rotate_right(uint16_t, uint8_t);
// event handlers
inline void event_encoder_left(uint16_t) __attribute__((always_inline));
inline void event_encoder_right(uint16_t) __attribute__((always_inline));
inline void event_switch_press_long(void) __attribute__((always_inline));
inline void event_switch_press_short(void) __attribute__((always_inline));
inline void event_switch_press_super_long(void) __attribute__((always_inline));
inline void event_switch_release(void) __attribute__((always_inline));
// Event FIFO ring buffer helpers
// inline struct Event_Queue * events_pop() __attribute__((always_inline));
void events_push(uint8_t event, uint16_t time);
// loop() breakouts
inline void process_event_queue(void) __attribute__((always_inline));
inline void process_last_events(void) __attribute__((always_inline));
inline void process_timed_switch_events(void) __attribute__((always_inline));
// utilities
inline uint8_t int_log2(uint16_t);
inline void reboot(void);


void setup()
{
  wdt_disable();
  acomp_disable();
  adc_disable();

  // Data Direction Register B - set as inputs ... aka _BV(PB3) aka = 0b00001000; to turn off, use &= ~(1 << PB3)
  DDRB |= (1 << PIN_ENCODER0);  // normally use PBx 
  DDRB |= (1 << PIN_ENCODER1);
  DDRB |= (1 << PIN_SWITCH);

  // Port B pullup register - reading this directly seems to cause a crash (?)
  PORTB |= (1 << PIN_ENCODER0);  // normally use PBx
  PORTB |= (1 << PIN_ENCODER1);
  PORTB |= (1 << PIN_SWITCH);

  // PCMSK: Pin Change Mask register for interrupts ; PCINTx: Pin Change INTerrupt #
  PCMSK |= (1 << PCINT0);
  PCMSK |= (1 << PCINT4);
  
  // GIMSK: General Interrupt Mask Register - PCIE: Pin Change Interrupt Enable bit  
  GIMSK |= (1 << PCIE);
  
  sei();
  
  pinb_history = PINB;
  
  if(!(PINB & SWITCH)) { // if the switch is depressed on startup, set it up
    state.switch_last_active = true;
    state.switch_press_short = true;
    state.switch_press_long = true;
  }
  
#ifdef DEBUG
  initTXPin();  // defaults to PB2; oddly, sets PB2 as input (?)
  delay(20);  
  Serial.print(F("\n\rsetup(); free="));
  Serial.println(freeMemory());
#endif

  uint16_t startup_mode = eeprom_read_word((const uint16_t *) Eeprom_Startup_Mode);
  
  if(startup_mode >> 8)  // System has been reset, or never setup
    mode = M0_Solid;     // TODO: default to solid mode
  else
    mode = (uint8_t) startup_mode & 0xF;
  
  uint16_t strip_length = eeprom_read_word((const uint16_t *) Eeprom_Strip_Length);
  
  if(strip_length == 0xFFFF)  // length has never been set
    strip_length = 1;  // TODO: consider a default of max length?

  if((mode & 0xF) == M0_Setup) {
    mode_state.setup.strip_length = strip_length;
  } else if((mode & 0xF) == M0_Solid) {
    eeprom_read_block((void *) &mode_state, (void *) Eeprom_M0_Solid, sizeof(mode_state));
  }
}


void loop()
{
  if(events.length) {
    process_event_queue();
#ifdef DEBUG
    if((mode & 0xF) == M0_Setup) {
      Serial.print(F("sLength:"));
      Serial.println((long) mode_state.setup.strip_length);
    } else if((mode & 0xF) == M0_Solid) {
      if((mode & 0xF0) == M1_Solid_Brightness) {
        for(uint8_t x = 0; x < mode_state.solid.brightness; x++)
          Serial.print(F("."));
        Serial.println();
      } else if((mode & 0xF0) == M1_Solid_Hue) {
        Serial.print(F("Hue:"));
        Serial.println(mode_state.solid.hue);
      } else if((mode & 0xF0) == M1_Solid_Saturation) {
        Serial.print(F("Sat:"));
        Serial.println(mode_state.solid.saturation);
      }
    }
#endif    
  }
  process_timed_switch_events();
  process_last_timeouts();
}


// TODO: add pin ground event
ISR(PCINT0_vect)
{
  if((PINB ^ pinb_history) & SWITCH) {
    events_push(PINB & SWITCH ? Switch_Up : Switch_Down, (uint16_t) millis());
  } else if((PINB ^ pinb_history) & ENCODER) {
    if(PINB & ENCODER_LEFT == ENCODER_LEFT_B && encoder_history == ENCODER_LEFT_A)
      events_push(Encoder_Left, (uint16_t) millis());
    else if(PINB & ENCODER_RIGHT == ENCODER_RIGHT_B && encoder_history == ENCODER_RIGHT_A)
      events_push(Encoder_Right, (uint16_t) millis());
    encoder_history = PINB & ENCODER;
  }
  pinb_history = PINB;
}


inline uint8_t encoder_accel(uint16_t time_diff)
{ // logarithmic dampened linear acceleration
  return ((ENCODER_SCALE_WINDOW - time_diff) / ENCODER_SCALE_FACTOR + 1) + 
         int_log2((ENCODER_SCALE_WINDOW - time_diff) | 2) / 2;
}
/*
inline uint8_t encoder_accel_linear(uint16_t time_diff)
{
  return (ENCODER_SCALE_WINDOW - time_diff) / ENCODER_SCALE_FACTOR + 1;
}

inline uint8_t encoder_accel_log(uint16_t time_diff)
{
  return int_log2((ENCODER_SCALE_WINDOW - time_diff) | 2);
}
*/

inline uint8_t encoder_rotate_left(uint16_t time_diff, uint8_t from_position) {
  if(last_encoder_left & LAST_ACTIVE && time_diff < ENCODER_SCALE_WINDOW) {
    uint8_t tmp_byte = encoder_accel(time_diff); 
    from_position -= from_position > tmp_byte ? tmp_byte : from_position;
  } else if(from_position > 0) {
    from_position--;
  }
  return from_position;
}

inline uint8_t encoder_rotate_right(uint16_t time_diff, uint8_t from_position) {
  if(last_encoder_right & LAST_ACTIVE && time_diff < ENCODER_SCALE_WINDOW) {
    uint8_t tmp_byte = encoder_accel(time_diff);     
    from_position += tmp_byte < 255 - from_position ? tmp_byte : 255 - from_position;
  } else if(from_position < 255) {
    from_position++;
  }
  return from_position;
}

inline void event_encoder_left(uint16_t event_time)
{
  uint16_t time_diff = event_time - last_encoder_left & ~LAST_ACTIVE;
  
  if((mode & 0xF) == M0_Setup) {
    mode_state.setup.strip_length = encoder_rotate_left(time_diff, mode_state.setup.strip_length);
  } else if((mode & 0xF) == M0_Solid) {
    if((mode & 0xF0) == M1_Solid_Brightness)
      mode_state.solid.brightness = encoder_rotate_left(time_diff, mode_state.solid.brightness);
    else if((mode & 0xF0) == M1_Solid_Hue)
      mode_state.solid.hue = encoder_rotate_left(time_diff, mode_state.solid.hue);
    else if((mode & 0xF0) == M1_Solid_Saturation)
      mode_state.solid.saturation = encoder_rotate_left(time_diff, mode_state.solid.saturation);    
  }
  
  last_encoder_left = (uint16_t) (event_time | LAST_ACTIVE);
  last_encoder_right &= ~LAST_ACTIVE;
}

inline void event_encoder_right(uint16_t event_time)
{
  uint16_t time_diff = event_time - last_encoder_right & ~LAST_ACTIVE;
  
  if((mode & 0xF) == M0_Setup) {
    mode_state.setup.strip_length = encoder_rotate_right(time_diff, mode_state.setup.strip_length);
  } else if((mode & 0xF) == M0_Solid) {
    if((mode & 0xF0) == M1_Solid_Brightness)
      mode_state.solid.brightness = encoder_rotate_right(time_diff, mode_state.solid.brightness);
    else if((mode & 0xF0) == M1_Solid_Hue)
      mode_state.solid.hue = encoder_rotate_right(time_diff, mode_state.solid.hue);
    else if((mode & 0xF0) == M1_Solid_Saturation)
      mode_state.solid.saturation = encoder_rotate_right(time_diff, mode_state.solid.saturation);    
  }
  
  last_encoder_right = (uint16_t) (event_time | LAST_ACTIVE);
  last_encoder_left &= ~LAST_ACTIVE;
}

inline void event_switch_press_short(void)
{
  Serial.println(F("shortPress"));
  if((mode & 0xF) == M0_Solid) {
    if((mode & 0xF0) == M1_Solid_Brightness)
      mode = M0_Solid | M1_Solid_Hue;
    else if((mode & 0xF0) == M1_Solid_Hue)
      mode = M0_Solid | M1_Solid_Saturation;
    else if((mode & 0xF0) == M1_Solid_Saturation)
      mode = M0_Solid | (uint16_t) M1_Solid_Brightness;
  }
}

inline void event_switch_press_long(void)
{
  Serial.println(F("longPress"));
}

inline void event_switch_press_super_long(void)
{
  if(state.switch_last_time == 0 && millis() < SWITCH_PRESS_SUPER_LONG_MS * 1.5) {
    Serial.println(F("setup..."));
    mode = M0_Setup;
  } else {
    Serial.println(F("saving..."));
    if((mode & 0x0F) == M0_Setup) {
      eeprom_update_word((uint16_t *) Eeprom_Strip_Length, mode_state.setup.strip_length);
    } else {
      eeprom_update_word((uint16_t *) Eeprom_Startup_Mode, (uint16_t) mode & (uint16_t) 0b1111);
      if((mode & 0xF) == M0_Solid)
        eeprom_update_block((void *) &mode_state, (void *) Eeprom_M0_Solid, sizeof(mode_state));
    }
    reboot();
  }
}

inline void event_switch_press_mega_long(void)
{
  if(state.switch_last_time == 0 && millis() < SWITCH_PRESS_MEGA_LONG_MS * 1.5) {
    Serial.println(F("reset!"));
    for(uint16_t *eeptr = (uint16_t *) 0; eeptr < Eeprom_End; eeptr++) {
      eeprom_update_word((uint16_t *) eeptr, (uint16_t) 0xFFFF);
    }
    reboot();
  }
}

inline void event_switch_release(void)
{}

/*
struct Event_Queue * events_pop()
{
  events.length--;
  uint8_t idx = events.start++;
  events.start %= MAX_EVENT_COUNT;
  return &events.queue[idx];
}
*/

void events_push(uint8_t event, uint16_t time)
{
  if(events.length == MAX_EVENT_COUNT)
    return;

  if(events.length == 0)
    events.start = 0;

  events.queue[(events.start + events.length++) % MAX_EVENT_COUNT] = (struct Event_Queue) {.event = event, .time = time};
}

// crude integer log2 formula with one decimal precision
inline uint8_t int_log2(uint16_t n) {
  uint8_t bit = 16;

  while(--bit > 0 && !(n & (1 << bit)));

  if(bit > 1 && n & (1 << (bit - 1)))  // check the next bit to round up if needed
    bit++;

  return bit;
}

// process interrupt event queue
inline void process_event_queue()
{
  while(events.length) {
    uint8_t tmp_event;
    uint16_t tmp_time;
    
    cli();  // disable interrupts while we pop an event off the event queue
    tmp_time = events.queue[events.start].time;
    tmp_event = events.queue[events.start++].event;
    events.start %= MAX_EVENT_COUNT;
    events.length--;    
    sei();

    // we need to recontsruct the 12 bit tmp.time into 15 bits
    uint16_t now = millis() & state_last_time_mask;
/*    
Serial.print(F("millis: "));
Serial.print((long) millis());
Serial.print(F(" now:"));
Serial.print((long) now);
Serial.print(F(" tt:"));
Serial.print((long) tmp_time);
*/
    if((now & event_queue_time_mask) >= tmp_time) {
      tmp_time = (now & ~event_queue_time_mask) + tmp_time;
//      Serial.print(F(" >= "));
    } else {
//      Serial.print(F(" < "));
      tmp_time = ((now & ~event_queue_time_mask) + tmp_time - event_queue_time_mask) & state_last_time_mask;
    }
/*    
Serial.print(F(" ltt:"));
Serial.println((long) tmp_time);
*/
    if(tmp_event == Encoder_Left) {
      event_encoder_left(tmp_time);
    } else if(tmp_event == Encoder_Right) {
      event_encoder_right(tmp_time);
    } else if(tmp_event == Switch_Down) {
      state.switch_last_active = true;
      state.switch_last_time = tmp_time;
      state.switch_press_short = false;
      state.switch_press_long = false;
      state.switch_press_super_long = false;
      state.switch_press_mega_long = false;
    } else if(tmp_event == Switch_Up) {
      state.switch_last_active = false;
      event_switch_release();
    }
  }
}

// expire 15-bit time counters before they rollover; this allows event tracking ~30 seconds
inline void process_last_timeouts(void)
{
  uint16_t now = millis() & state_last_time_mask;
  
  if(last_encoder_left & LAST_ACTIVE && ((uint16_t) millis() - last_encoder_left & ~LAST_ACTIVE) > 31000) {
    last_encoder_left &= ~LAST_ACTIVE;
  }
  if(last_encoder_right & LAST_ACTIVE && ((uint16_t) millis() - last_encoder_right & ~LAST_ACTIVE) > 31000) {
    last_encoder_right &= ~LAST_ACTIVE;
  }

  if(state.switch_last_active) {
/*    delay(50);
    Serial.print(F("now: "));
    Serial.print((long) now);
    Serial.print(F(" slt: "));
    Serial.print((long) state.switch_last_time);
    Serial.print(F(" diff: "));
    Serial.println((long) (now - state.switch_last_time) & state_last_time_mask);
*/
    if(state.switch_last_active && ((now - state.switch_last_time) & state_last_time_mask) > 31000)
      state.switch_last_active = false;
  }
}

inline void process_timed_switch_events()
{
  if(state.switch_last_active) { // switch is down and has not rolled over
    uint16_t time_diff = (uint16_t) millis() - state.switch_last_time;
/*Serial.print((long) millis()); // & state_last_time_mask);
Serial.print(F(" / "));
Serial.print((long) state.switch_last_time);
Serial.print(F(" / "));
Serial.println((long) time_diff);*/
    if(!state.switch_press_short) {
      state.switch_press_short = true;
      event_switch_press_short();
    }
    if(!state.switch_press_long && time_diff >= SWITCH_PRESS_LONG_MS) {
      state.switch_press_long = true;
      event_switch_press_long();
    }
    if(!state.switch_press_super_long && time_diff >= SWITCH_PRESS_SUPER_LONG_MS) {
      state.switch_press_super_long = true;
      event_switch_press_super_long();
    }
    if(!state.switch_press_mega_long && time_diff >= SWITCH_PRESS_MEGA_LONG_MS) {
      state.switch_press_mega_long = true;
      event_switch_press_mega_long();
    }
  }
}

inline void reboot(void)
{
  cli();
  WDTCR = 0xD8 | WDTO_1S;  // TODO: tune
  sei();
  wdt_reset();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode(); 
}
