/* 
 * ATtiny85 on macOS  -> https://gist.github.com/gurre/20441945fdcbb0c2f2c346d9f894a361
 * Digispark Basics   -> http://digistump.com/wiki/digispark/tutorials/basics
 * Digispark Tutorial -> http://digistump.com/wiki/digispark/tutorials/connecting
 * ATtiny85           -> https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf
 * 
 * 0 -> PB0 MOSI/DI/SDA/AIN0/OC0A/OC1A/AREF/PCINT0 
 * 1 -> PB1 MISO/DO/AIN1/OC0B/OC1A/PCINT1            <- onboard LED
 * 2 -> PB2 SCK/USCK/SCL/ADC1/T0/INT0/PCINT2         <- default serial for ATtinySerialOut
 * 3 -> PB3 PCINT3/XTAL1/CLKI/OC1B/ADC3              <- USB comm at boot
 * 4 -> PB4 PCINT4/XTAL2/CLKO/OC1B/ADC2              <- USB comm at boot
 * 5 -> PB5 PCINT5/RESET/ADC0/dW
 * 
 * APA102 - uses 2-wire SPI      -> https://cpldcpu.wordpress.com/2014/08/27/apa102/  
 * Pololu APA102 lib w/ ATTiny85 https://gist.github.com/beriberikix/db669e29c92935c71a9a
 * WS2812 -> uses 800 khz signal
 * I2C -> https://www.instructables.com/id/Using-an-I2C-LCD-on-Attiny85/
 * EEPROM -> https://www.avrfreaks.net/forum/solved-attiny85-eeprom
 *   // should do 100k write cycles (conservatively) ; save settings after inactivity timer.
 * SLEEP -> https://arduino.stackexchange.com/questions/61721/attiny85-reset-itself-instead-of-wakeup-procedure
 *       -> https://gist.github.com/JChristensen/5616922
 *       -> http://www.technoblogy.com/show?KX0
 * Attiny85 takes ~.5uA in power-down mode :)
 * Interrupts -> https://www.avrfreaks.net/forum/triggering-isrpcint0vect-interrupts
 * Globals may be the fastest due to absolute pointers rather than relative to stack https://forum.arduino.cc/index.php?topic=473309.0
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>          // for uint*_t

#include "encoder_event.h"


#define DEBUG

// used with DEBUG
#include "ATtinySerialOut.h"  // https://github.com/ArminJo/ATtinySerialOut (defaults to PB2)
#include "memoryFree.h"       // for memoryFree();


// Pin assignments
#define PIN_ENCODER0   PB0
#define PIN_ENCODER1   PB1
#define PIN_PCB_LED    PB1
#define PIN_TINYSERIAL PB2
#define PIN_SWITCH     PB4

// Encoder & switch pinout masks
#define MASK_ENCODER0 (1 << PIN_ENCODER0)
#define MASK_ENCODER1 (1 << PIN_ENCODER1)
#define MASK_ENCODER (MASK_ENCODER0 | MASK_ENCODER1)
#define MASK_SWITCH (1 << PIN_SWITCH)

// Encoder direction masks
#define ENCODER_DIR0A MASK_ENCODER1
#define ENCODER_DIR0B MASK_ENCODER0
#define ENCODER_DIR1A (MASK_ENCODER0 & MASK_ENCODER1)
#define ENCODER_DIR1B (MASK_ENCODER0 | MASK_ENCODER1)

#define adc_disable() (ADCSRA &= ~(1<<ADEN))  // disable ADC (before power-off) as it uses ~320uA
#define acomp_disable() (ACSR |= _BV(ACD))    // disable analog comparator

struct encoder_event * volatile encoder_events;  // linked list for encoder FIFO
volatile uint8_t pinb_history;                   // tracks history to compare last event

void setup() {
#ifdef DEBUG
  initTXPin();  // defaults to PB2; oddly, sets PB2 as input (?)
  delay(20);  
  Serial.println(F("\n\n\n\n\rsetup()\n\r-------"));
  Serial.print(F("- free_memory:   ")); Serial.println(freeMemory());
  Serial.print(F("- MASK_ENCODER0: ")); Serial.println(MASK_ENCODER0, BIN);
  Serial.print(F("- MASK_ENCODER1: ")); Serial.println(MASK_ENCODER1, BIN);
  Serial.print(F("- MASK_ENCODER:  ")); Serial.println(MASK_ENCODER, BIN);
  Serial.print(F("- MASK_SWITCH:   ")); Serial.println(MASK_SWITCH, BIN);
#endif


  // Data Direction Register B - set as inputs ... aka _BV(PB3) aka = 0b00001000; to turn off, use &= ~(1 << PB3)
  DDRB |= (1 << PB0);
  DDRB |= (1 << PB1);
  DDRB |= (1 << PB4);

  // Port B pullup register - reading this directly seems to cause a crash (?)
  PORTB |= (1 << PB0);
  PORTB |= (1 << PB1);
  PORTB |= (1 << PB4);
//   PORTB |= (1 << PB5);

  // PCMSK: Pin Change Mask register for interrupts ; PCINT*: Pin Change INTerrupt #
  PCMSK |= (1 << PCINT0);
  PCMSK |= (1 << PCINT4);
  
  // GIMSK: General Interrupt Mask Register - PCIE: Pin Change Interrupt Enable bit  
  GIMSK |= (1 << PCIE);
  
  sei();
  
  pinb_history = PINB;
}

void loop() {
  struct encoder_event *tmp_event;

  if(encoder_events) {
    cli();
#ifdef DEBUG
    uint8_t queue_length = 1;
    tmp_event = encoder_events;
    while(tmp_event->next) {
      queue_length++;
      tmp_event = tmp_event->next;
    }
#endif

    tmp_event = encoder_events;
    encoder_events = encoder_events->next;
    sei();

    if((tmp_event->pinb ^ pinb_history) & MASK_SWITCH) {
      if(tmp_event->pinb & MASK_SWITCH)
        Serial.println("Switch release");
      else
        Serial.println("Switch press");
    } else if((tmp_event->pinb ^ pinb_history) & MASK_ENCODER0) {
      switch(tmp_event->pinb & MASK_ENCODER) {
        case ENCODER_DIR0A:
          Serial.println(F("Encoder left (rising)"));
          break;
        case ENCODER_DIR0B:
          Serial.println(F("Encoder left (falling)"));
          break;
        case ENCODER_DIR1A:
          Serial.println(F("Encoder right (rising)"));
          break;
        case ENCODER_DIR1B:
          Serial.println(F("Encoder right (falling)"));
          break;
      }
    }
    
//    uint8_t changed_bits = (tmp_event->pinb ^ pinb_history); //  & (MASK_SWITCH | MASK_ENCODER0);
     
#ifdef DEBUGx 
    Serial.print(F("loop() time_now:")); Serial.print(millis());
    Serial.print(F(" free_memory: "));   Serial.print(freeMemory()); 
    Serial.print(F(" queue_length: "));  Serial.println(queue_length);   
    encoder_event_print(tmp_event);
//    Serial.print(F("- pinb_history: "));   Serial.println(pinb_history, BIN); 
//    Serial.print(F("- changed_bits: "));   Serial.println(changed_bits, BIN); 
#endif
    pinb_history = tmp_event->pinb;
    free(tmp_event);
  }
}


ISR(PCINT0_vect) {
  // This logic is going to save on *all* interrupts; we should filter
  struct encoder_event * event = encoder_events;
  struct encoder_event * new_event = (struct encoder_event*)malloc(sizeof(struct encoder_event));
  
  if(new_event) {
    new_event->time = (uint16_t) millis();
    new_event->pinb = (uint8_t) PINB;
    new_event->next = 0;
    cli();
    if(event) {
      while(event->next) event = event->next;
      event->next = new_event;
    } else {
      encoder_events = new_event;
    }
    sei();
  }
}


/*
  adc_disable();
  acomp_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  sleep_cpu();
*/
