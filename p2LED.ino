/* free_memory() => 447 (before transition to state struct)
 *               => 454 using Events ring buffer and state struct for switch
 *               => 457 moved to mode state union
 *               => 460 combined encoder buffers
 *               
 * need 432 bytes for LED HSV[144] buffer (high density 1m strips), plus memory for effect processing
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

// For tuning encoder acceleration
#define ENCODER_SCALE_WINDOW 60  // acceleration window in ms
#define ENCODER_SCALE_FACTOR 10  // max linear accelleration = window / factor

#define LED_STRIP_MAX_LENGTH 144

// Interrupt event queue, minimize time spent in interrupt handler
#define MAX_EVENT_COUNT 4  // how many events to queue, TODO: tune (reduce & backoff)

enum Event_Types {
  Encoder_Left,
  Encoder_Right,
  Switch_Down,
  Switch_Up,
};

const uint16_t event_queue_time_mask = 0b111111111111;  // (& event_queue_time_mask) == (% 0x1000)

struct Event_Queue {
  uint16_t event : 4;
  uint16_t time : 12;  // we could get by with only 8 bits here (255ms)
};

// FIFO ring buffer; push to end, pop from front
struct Events {  // sizeof() = 1 + 2 * MAX_EVENT_COUNT
  uint8_t start : 4;
  uint8_t length : 4;
  struct Event_Queue queue[MAX_EVENT_COUNT];
} volatile events;

// if we was CRaZaaYYY we could turn these into two 4 bit bit fields with additional processing
volatile uint8_t pinb_history;     // to determine which pin change event caused an interrupt
volatile uint8_t encoder_history;  // to store first phase of quadrature events

#define SWITCH_DOUBLE_PRESS_MS 250       // ms between presses to count as a double press
#define SWITCH_PRESS_LONG_MS 750         // ms to count as a long switch event
#define SWITCH_PRESS_SUPER_LONG_MS 2000  // ms to count as a super long switch event
#define SWITCH_PRESS_MEGA_LONG_MS 5000   // ms to mega long (reset event)
#define LAST_ACTIVE_TIMEOUT_MS 15750     // max length a "last state" can be active (14-bit == 16384 - headroom)

const uint16_t last_active_timeout_mask = 0b11111111111111;  // (& last_active_timeout_mask) == (% 0x4000)

struct State {
  uint16_t switch_last_active : 1;
  uint16_t switch_double_active : 1;
  uint16_t switch_last_time : 14;
  uint16_t encoder_left_active : 1;
  uint16_t encoder_right_active : 1;
  uint16_t encoder_last_time : 14;
} state;

enum Modes {
  Mode_Default = 0,
  Mode_Setup = 0,
  Mode_Solid,
  Mode_Wheel,
  Mode_Breathe,
  Mode_End,  // *_End used as a marker so we can simple increment through modes
  Mode_Setup_Length = 0,
  Mode_Setup_Reserved,
  Mode_Setup_End,
  Mode_Solid_Brightness = 0,
  Mode_Solid_Hue,
  Mode_Solid_Saturation,
  Mode_Solid_End,
  Mode_Wheel_Brightness = 0,
  Mode_Wheel_Speed,
  Mode_Wheel_Saturation,
  Mode_Wheel_End,
  Mode_Breathe_Brightness = 0,
  Mode_Breathe_Speed,
  Mode_Breathe_Hue,
  Mode_Breathe_Saturation,
  Mode_Breathe_End,
};

const uint8_t PROGMEM mode_level1_max[Mode_End][Mode_Breathe_End + 1] = { 
  {
    LED_STRIP_MAX_LENGTH, // Mode_Setup_Length
    0,
    0,
    0,
    0,
  }, {
    255,  // Mode_Solid_Brightness
    255,  // Mode_Solid_Hue
    255,  // Mode_Solid_Saturation
    0,
    0,
  }, {
    255,  // Mode_Wheel_Brightness
    255,  // Mode_Wheel_Speed
    255,  // Mode_Wheel_Saturation
    0,
    0,
  }, {
    255,  // Mode_Breathe_Brightness
    255,  // Mode_Breathe_Speed
    255,  // Mode_Breathe_Hue
    255,  // Mode_Breathe_Saturation
    0,
  },
};

union Mode_State {
  struct {
    uint16_t strip_length;  // only used during read/write from EEPROM
    uint8_t startup_mode;
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
  } breathe;
  uint8_t setting[Mode_Breathe_End];
};
  
struct Mode {  // for tracking operation mode; should this be inside of state?
  uint8_t level0 : 4;  // Level 0
  uint8_t level1 : 4;  // Level 1
  union Mode_State state;
} mode;

// EEPROM map
const uint16_t * Eeprom_Strip_Length = (uint16_t *) 0x00;
const uint8_t * Eeprom_Startup_Mode = (uint8_t *) 0x02;
const uint8_t Eeprom_Sizeof_Mode_Setting = 0x10;  // Allow for more than just the union for future growth
const uint16_t * Eeprom_End = (uint16_t *) (Mode_End * Eeprom_Sizeof_Mode_Setting);

// encoder functions
inline uint8_t encoder_accel(uint16_t) __attribute__((always_inline));
inline uint8_t encoder_rotate_left(uint16_t, uint8_t, uint8_t);
inline uint8_t encoder_rotate_right(uint16_t, uint8_t, uint8_t);
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
inline void process_timed_events(void);
// utilities
inline uint16_t get_last_time_diff(uint16_t);
inline uint8_t int_log2(uint16_t);
inline void reboot(void);
inline void eeprom_read_global_settings() __attribute__((always_inline));
inline void eeprom_read_mode_settings();
inline void eeprom_reset();
inline void eeprom_write_mode_settings();
inline void eeprom_write_setup_settings();

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
  
  sei();  // SEt Interrupt instruction
  
  pinb_history = PINB;
  
  if(!(PINB & SWITCH)) { // if the switch is depressed on startup, set the state up
    state.switch_last_active = true;
  }
  
#ifdef DEBUG
  initTXPin();  // defaults to PB2; oddly, sets PB2 as input (?)
  delay(20);  
  Serial.print(F("\n\rsetup(); free="));
  Serial.println(freeMemory());

  for(uint8_t i = 0; i < Mode_End; i++) {
    for(uint8_t j = 0; j < Mode_Breathe_End + 1; j++) {
      Serial.print(F("["));
      Serial.print(i);
      Serial.print(F("]["));
      Serial.print(j);
      Serial.print(F("] = "));
      Serial.println(pgm_read_byte(&mode_level1_max[i][j]));
    }
  }
  for(uint8_t i = 0; i < Mode_End; i++) {
    for(uint8_t * eeptr = (uint8_t *) (i * 0x10); eeptr < (uint8_t *) ((i + 1) * 0x10); eeptr++) {
      Serial.print(eeprom_read_byte(eeptr), HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
#endif

  eeprom_read_global_settings();
  eeprom_read_mode_settings();
}


void loop()
{
  if(events.length) {
    process_event_queue();
#ifdef DEBUG
  Serial.print(mode.level0);
  Serial.print(F("."));
  Serial.print(mode.level1);
  Serial.print(F(": "));
  Serial.println(mode.state.setting[mode.level1]);
/*  if(mode.level1 == Mode_Solid_Brightness) {
    for(uint8_t x = 0; x < mode.state.solid.brightness; x++)
      Serial.print(F("."));
    Serial.println(); */
#endif    
  }
  process_timed_events();
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


inline void eeprom_read_global_settings()
{
  uint16_t strip_length = eeprom_read_word((uint16_t *) Eeprom_Strip_Length);
  
  if(strip_length > LED_STRIP_MAX_LENGTH)  // System has been reset, or never setup
    strip_length = LED_STRIP_MAX_LENGTH;

  mode.level0 = eeprom_read_byte((uint8_t *) Eeprom_Startup_Mode);
  
  if(mode.level0 >= Mode_End)  // System has been reset, or never setup
    mode.level0 = Mode_Solid;
 
#ifdef DEBUG
  Serial.print(F("sLength: "));
  Serial.print((long) strip_length);
  Serial.print(F(" Mode: "));
  Serial.println(mode.level0);
#endif
}

inline void eeprom_read_mode_settings()
{
  eeprom_read_block((void *) &mode.state,
                    (void *) ((uint16_t) mode.level0 * Eeprom_Sizeof_Mode_Setting),
                    sizeof(Mode_State));
}

inline void eeprom_reset()
{
  for(uint8_t * eeptr = (uint8_t *) 0; eeptr < (uint8_t *) Eeprom_End; eeptr++) {
    eeprom_update_byte(eeptr, 0xFF);
  }
}

inline void eeprom_write_setup_settings()
{
  eeprom_update_word((uint16_t *) Eeprom_Strip_Length,
                     (uint16_t) mode.state.setup.strip_length);
}

inline void eeprom_write_mode_settings()
{
  eeprom_update_byte((uint8_t *) Eeprom_Startup_Mode,
                     (uint8_t) mode.level0);
  eeprom_update_block((void *) &mode.state,
                      (void *) ((uint16_t) mode.level0 * Eeprom_Sizeof_Mode_Setting),
                      sizeof(Mode_State));
}

inline uint8_t encoder_accel(uint16_t time_diff)
{ // logarithmic dampened linear acceleration
  return ((ENCODER_SCALE_WINDOW - time_diff) / ENCODER_SCALE_FACTOR + 1) + 
         int_log2((ENCODER_SCALE_WINDOW - time_diff) | 2) / 2;
}

inline uint8_t encoder_rotate_left(uint16_t time_diff, uint8_t value, uint8_t min) {
  if(state.encoder_left_active && time_diff < ENCODER_SCALE_WINDOW) {
    uint8_t tmp_byte = encoder_accel(time_diff); 
    value -= value - min > tmp_byte ? tmp_byte : value - min;
  } else if(value > min) {
    value--;
  }
  return value;
}

inline uint8_t encoder_rotate_right(uint16_t time_diff, uint8_t value, uint8_t max) {
  if(state.encoder_right_active && time_diff < ENCODER_SCALE_WINDOW) {
    uint8_t tmp_byte = encoder_accel(time_diff);     
    value += tmp_byte < max - value ? tmp_byte : max - value;
  } else if(value < max) {
    value++;
  }
  return value;
}

inline void event_encoder_left(uint16_t event_time)
{
  uint16_t time_diff = event_time - state.encoder_last_time;

  mode.state.setting[mode.level1] = encoder_rotate_left(time_diff,
                                                        mode.state.setting[mode.level1],
                                                        0);
  state.encoder_last_time = event_time;
  state.encoder_left_active = true;
  state.encoder_right_active = false;
}

inline void event_encoder_right(uint16_t event_time)
{
  uint16_t time_diff = event_time - state.encoder_last_time;

  mode.state.setting[mode.level1] = encoder_rotate_right(time_diff,
                                                         mode.state.setting[mode.level1],
                                                         pgm_read_byte(&(mode_level1_max[mode.level0][mode.level1])));
                                                         // mode_ranges[mode.level0][mode.level1].max);
  state.encoder_last_time = event_time;
  state.encoder_right_active = true;
  state.encoder_left_active = false;
}

inline void event_switch_double_press(void)
{
#ifdef DEBUG
  Serial.println(F("doublePress"));  // This could be made more generic by incrementing the mode, and checking rollover
#endif
  if(mode.level0 >= Mode_Solid) {
    if(++mode.level0 == Mode_End)
      mode.level0 = Mode_Solid;
      
    mode.level1 = Mode_Default;
    eeprom_read_mode_settings();
  }
}

inline void event_switch_press_short(void)
{
#ifdef DEBUG
  Serial.println(F("shortPress"));  // This could be made more generic by incrementing the mode, and checking rollover
#endif
 if(!pgm_read_byte(&(mode_level1_max[mode.level0][++mode.level1])))
    mode.level1 = Mode_Default;
}

inline void event_switch_press_long(void)
{
#ifdef DEBUG
  Serial.println(F("longPress"));  // This could be made more generic by incrementing the mode, and checking rollover
#endif 
  mode.level1 = Mode_Default;
}

inline void event_switch_press_super_long(void)
{
  uint16_t mils = millis();  // TODO: unneeded when out of debug

#ifdef DEBUG
  Serial.print(F("superLongPress - s_l_t: "));  // This could be made more generic by incrementing the mode, and checking rollover
  Serial.print(state.switch_last_time);
  Serial.print(F(" millis(): "));
  Serial.println(mils);
#endif  
  if(state.switch_last_time == 0 && mils < last_active_timeout_mask) {
#ifdef DEBUG
    Serial.println(F("setup..."));
#endif
    mode.level0 = Mode_Setup;
    if(mode.state.setup.strip_length > LED_STRIP_MAX_LENGTH)  // System has been reset, or never setup
      mode.state.setup.strip_length = LED_STRIP_MAX_LENGTH;
  } else {
#ifdef DEBUG
    Serial.println(F("saving..."));
#endif
    if(mode.level0 == Mode_Setup) {
      eeprom_write_setup_settings();
    } else {
      eeprom_write_mode_settings();
    }
    reboot();
  }
}

inline void event_switch_press_mega_long(void)
{
  uint16_t mils = millis();  // TODO: unneeded when out of debug
#ifdef DEBUG
  Serial.println(F("megaLongPress - s_l_t: "));  // This could be made more generic by incrementing the mode, and checking rollover
  Serial.print(state.switch_last_time);
  Serial.print(F(" millis(): "));
  Serial.println(mils);
#endif  
  if(state.switch_last_time == 0 && mils < last_active_timeout_mask) {
#ifdef DEBUG
    Serial.println(F("reset!"));
#endif
    eeprom_reset();
    reboot();
  }
}

inline void event_switch_release(void)
{
#ifdef DEBUG
  Serial.print(F("switch_last_active: "));
  Serial.print(state.switch_last_active);
  Serial.print(F(" switch_double_active: "));
  Serial.print(state.switch_double_active);
  Serial.print(F(" now: "));
  Serial.print(millis(), DEC);
  Serial.print(F(" state.switch_last_time: "));
  Serial.print(state.switch_last_time, DEC);
  Serial.print(F(" get_last_time_diff: "));
  Serial.println((long) get_last_time_diff(state.switch_last_time));
#endif
  if(state.switch_last_active) { // switch is down and last active timer has not rolled over
    if(state.switch_double_active) {
      event_switch_double_press();
      
      state.switch_last_active = false;
      state.switch_double_active = false;
    } else {
      if(get_last_time_diff(state.switch_last_time) >= SWITCH_PRESS_MEGA_LONG_MS)
        event_switch_press_mega_long();
      else if(get_last_time_diff(state.switch_last_time) >= SWITCH_PRESS_SUPER_LONG_MS)
        event_switch_press_super_long();
      else if(get_last_time_diff(state.switch_last_time) >= SWITCH_PRESS_LONG_MS)
        event_switch_press_long();
      else
        event_switch_press_short();
        
      state.switch_last_active = false;
      state.switch_double_active = true;
    }
    state.switch_last_time = millis() & last_active_timeout_mask;
  }
}

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

  events.queue[(events.start + events.length) % MAX_EVENT_COUNT].event = event;
  events.queue[(events.start + events.length++) % MAX_EVENT_COUNT].time = time;
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
    // uint16_t now;
    
    cli();  // disable interrupts while we pop an event off the event queue
    tmp_time = events.queue[events.start].time;
    tmp_event = events.queue[events.start++].event;
    events.start %= MAX_EVENT_COUNT;
    events.length--;    
    sei();

    uint16_t now = millis() & last_active_timeout_mask;

    // we need to reconstruct the 12 bit tmp.time into 14/15 bits
    if((now & event_queue_time_mask) >= tmp_time)
      tmp_time = (now & ~event_queue_time_mask) + tmp_time;
    else
      tmp_time = ((now & ~event_queue_time_mask) + tmp_time - event_queue_time_mask) & last_active_timeout_mask;
       
    if(tmp_event == Encoder_Left || tmp_event == Encoder_Right) {
      if(tmp_event == Encoder_Left)
        event_encoder_left(tmp_time);
      else
        event_encoder_right(tmp_time); 
    } else if(tmp_event == Switch_Down) {
      state.switch_last_active = true;
      state.switch_last_time = tmp_time;
    } else if(tmp_event == Switch_Up) {
      event_switch_release();
      state.switch_last_active = false;
    }
  }
}

inline uint16_t get_last_time_diff(uint16_t last_time) {
  return ((millis() - last_time) & last_active_timeout_mask);
}
  
inline void process_timed_events()
{
  // expire time counters before they rollover
  if(state.encoder_left_active && get_last_time_diff(state.encoder_last_time) > LAST_ACTIVE_TIMEOUT_MS)
    state.encoder_left_active = false;

  if(state.encoder_right_active && get_last_time_diff(state.encoder_last_time) > LAST_ACTIVE_TIMEOUT_MS)
    state.encoder_right_active = false;   
  
  if(state.switch_last_active && get_last_time_diff(state.switch_last_time) > LAST_ACTIVE_TIMEOUT_MS)
    state.switch_last_active = false;

  if(state.switch_double_active && get_last_time_diff(state.switch_last_time) > SWITCH_DOUBLE_PRESS_MS)
    state.switch_double_active = false;
}

inline void reboot(void)
{
  cli();
  WDTCR = 0xD8 | WDTO_1S;  // TODO: tune to shortest timer that consistently works
  sei();
  wdt_reset();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode(); 
}
