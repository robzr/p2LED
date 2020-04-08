/* free_memory() => 459 
 * need 432 bytes for HSV[144] buffer to accomodate high density 1m strips
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#define DEBUG

// used with DEBUG
#include "ATtinySerialOut.h"  // https://github.com/ArminJo/ATtinySerialOut (defaults to PB2)
#include "memoryFree.h"       // for memoryFree();

#define adc_disable() (ADCSRA &= ~(1<<ADEN))  // disable ADC (before power-off) as it uses ~320uA
#define acomp_disable() (ACSR |= _BV(ACD))    // disable analog comparator

// Pin assignments
#define PIN_ENCODER0   PB0
#define PIN_ENCODER1   PB1
#define PIN_PCB_LED    PB1
#define PIN_TINYSERIAL PB2
#define PIN_SWITCH     PB4

// Encoder & switch bitmasks to apply to PINB register
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
#define ENCODER_SCALE_WINDOW 100  // ms for acceleration window
#define ENCODER_SCALE_FACTOR 10   // divisor that determines acceleration rate

// Interrupt event queue
#define MAX_EVENT_COUNT 6  // how many events to queue, TODO: tune (reduce & backoff)

enum Events {
  Encoder_Left,
  Encoder_Right,
  Switch_Down,
  Switch_Up
};

struct event {
  uint16_t time;
  uint8_t event;
} volatile events[MAX_EVENT_COUNT];

volatile uint8_t event_count = 0;
volatile uint8_t pinb_history;     // to determine which pin change event caused an interrupt
volatile uint8_t encoder_history;  // to store first phase of quadrature events

// last_* use the lower 15 bits to track 32 seconds of time; the top bit to determine state
#define LAST_ACTIVE (uint16_t) (1 << 15)
uint16_t last_encoder_left = 0;
uint16_t last_encoder_right = 0;
uint16_t last_switch_down = 0;

uint8_t brightness = 128;


inline void process_event_queue() __attribute__((always_inline));
inline void process_last_timeouts() __attribute__((always_inline));


void setup() {
  adc_disable();
  acomp_disable();

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
  last_switch_down &= PINB & SWITCH;
  
#ifdef DEBUG
  initTXPin();  // defaults to PB2; oddly, sets PB2 as input (?)
  delay(20);  
  Serial.print(F("\n\n\n\n\rsetup(); free_memory() == "));
  Serial.println(freeMemory());
#endif
}


void loop() {
  if(event_count) {
    process_event_queue();
#ifdef DEBUG
    for(uint8_t x = 0; x < brightness; x++)
      Serial.print(F("."));
    Serial.println();
#endif    
  }

  process_last_timeouts();
}

// process interrupt event queue
inline void process_event_queue() {
  while(event_count) {
    uint8_t tmp_byte = 0;
    uint8_t tmp_event; 
    uint16_t tmp_time;

    cli();  // disable interrupts while we pop an event off the event queue
    tmp_time = events[0].time;
    tmp_event = events[0].event;
    do {
      events[tmp_byte].time = events[tmp_byte + 1].time;
      events[tmp_byte].event = events[tmp_byte + 1].event;
    } while(++tmp_byte < event_count - 1);
    event_count--;
    sei();
      
    if(tmp_event == Encoder_Left) {
      if(last_encoder_left & LAST_ACTIVE && (tmp_time - last_encoder_left & ~LAST_ACTIVE) < ENCODER_SCALE_WINDOW) {
        tmp_byte = (ENCODER_SCALE_WINDOW - (tmp_time - last_encoder_left & ~LAST_ACTIVE)) / ENCODER_SCALE_FACTOR + 1;
        brightness -= brightness > tmp_byte ? tmp_byte : brightness;
      } else if(brightness > 0)
        brightness--;
      last_encoder_left = (uint16_t) (tmp_time | LAST_ACTIVE);
      last_encoder_right &= ~LAST_ACTIVE;
    } else if(tmp_event == Encoder_Right) {
      if(last_encoder_right & LAST_ACTIVE && (tmp_time - last_encoder_right & ~LAST_ACTIVE) < ENCODER_SCALE_WINDOW) {
        tmp_byte = (ENCODER_SCALE_WINDOW - (tmp_time - last_encoder_right & ~LAST_ACTIVE)) / ENCODER_SCALE_FACTOR + 1;     
        brightness += tmp_byte < 255 - brightness ? tmp_byte : 255 - brightness;
      } else if(brightness < 255)
        brightness++;
      last_encoder_right = (uint16_t) (tmp_time | LAST_ACTIVE);
      last_encoder_left &= ~LAST_ACTIVE;
    } else if(tmp_event == Switch_Down) {
      last_switch_down = LAST_ACTIVE | tmp_time;
      Serial.print(F("Switch Pressed - event time: "));
      Serial.println(last_switch_down & ~LAST_ACTIVE);
    } else if(tmp_event == Switch_Up) {
      last_switch_down &= ~LAST_ACTIVE;
      Serial.print(F("Switch Released - event time: "));
      Serial.print((uint16_t) tmp_time & ~LAST_ACTIVE);
      Serial.print(F(" down time: "));
      Serial.println(tmp_time - last_switch_down & ~LAST_ACTIVE);
    }
  }
}

// expire 15-bit time counters before they rollover; this allows event tracking ~30 seconds
inline void process_last_timeouts() {
  if(last_encoder_left & LAST_ACTIVE && ((uint16_t) millis() - last_encoder_left) & ~LAST_ACTIVE > 31000)
    last_encoder_left &= ~LAST_ACTIVE;
  if(last_encoder_right & LAST_ACTIVE && ((uint16_t) millis() - last_encoder_right) & ~LAST_ACTIVE > 31000)
    last_encoder_right &= ~LAST_ACTIVE;
  if(last_switch_down & LAST_ACTIVE && ((uint16_t) millis() - last_switch_down) & ~LAST_ACTIVE > 31000)
    last_switch_down &= ~LAST_ACTIVE;
}

// TODO: add pin ground event
ISR(PCINT0_vect) {
  if(event_count < MAX_EVENT_COUNT - 1) {
    if((PINB ^ pinb_history) & SWITCH) {
      events[event_count].time = (uint16_t) millis();    
      if(PINB & SWITCH)
        events[event_count++].event = Switch_Up;
      else
        events[event_count++].event = Switch_Down;
    } else if((PINB ^ pinb_history) & ENCODER) {
      events[event_count].time = (uint16_t) millis();      
      if(PINB & ENCODER_LEFT == ENCODER_LEFT_B && encoder_history == ENCODER_LEFT_A)
        events[event_count++].event = Encoder_Left;
      else if(PINB & ENCODER_RIGHT == ENCODER_RIGHT_B && encoder_history == ENCODER_RIGHT_A)
        events[event_count++].event = Encoder_Right;
      encoder_history = PINB & ENCODER;
    }
  }
  pinb_history = PINB;
}
