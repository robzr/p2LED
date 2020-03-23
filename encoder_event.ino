#include "encoder_event.h"
#ifdef DEBUG
#include "ATtinySerialOut.h"
#endif

#ifdef DEBUG
void encoder_event_dump(struct encoder_event *encoder_event) {
  while(encoder_event) {
    encoder_event_print(encoder_event);
    encoder_event = encoder_event->next;
  }
}
#endif

struct encoder_event* encoder_event_new(uint16_t time, uint8_t pinb) {
  struct encoder_event *encoder_event = (struct encoder_event*)malloc(sizeof(struct encoder_event));
  encoder_event->time = time;
  encoder_event->pinb = pinb;
  encoder_event->next = 0;
  return encoder_event;
}

struct encoder_event* encoder_event_pop() {
  struct encoder_event *encoder_event = encoder_events;
  encoder_events = encoder_event->next;
  return encoder_event;
}

#ifdef DEBUG
void encoder_event_print(struct encoder_event *encoder_event) {
  Serial.print(F("- time: "));
  Serial.print((unsigned int)encoder_event->time, DEC);
  Serial.print(F(" next: "));
  Serial.print((unsigned int)encoder_event->next, HEX);
  Serial.print(F(" pinb: "));
  Serial.println((unsigned short int)encoder_event->pinb, BIN);
}
#endif

void encoder_event_push(uint16_t time, uint8_t pinb) {
  struct encoder_event *encoder_event = encoder_events;

// TODO: compare size
#ifdef COMPACT_LL
  while(encoder_events && encoder_event->next) encoder_event = encoder_event->next;
  *(encoder_events ? &encoder_event->next : &encoder_events) = encoder_event_new(time, pinb);
#else
  if(encoder_events) {
    while(encoder_event->next) encoder_event = encoder_event->next;
    encoder_event->next = encoder_event_new(time, pinb);
  } else
    encoder_events = encoder_event_new(time, pinb);
#endif
}
