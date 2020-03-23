#include <stdint.h>
#include <stdlib.h>

#ifndef ENCODER_EVENT_H
#define ENCODER_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

struct encoder_event {
  uint16_t time;
  uint8_t pinb;
  struct encoder_event* next;
};

#ifdef DEBUG
inline void encoder_event_dump(struct encoder_event *encoder_event) __attribute__((always_inline));
inline void encoder_event_print(struct encoder_event *encoder_event) __attribute__((always_inline));
#endif

inline struct encoder_event* encoder_event_new(uint16_t time, uint8_t pinb) __attribute__((always_inline));
inline struct encoder_event* encoder_event_pop() __attribute__((always_inline)); // don't forget to free() after using it
inline void encoder_event_push(uint16_t time, uint8_t pinb) __attribute__((always_inline));

#ifdef  __cplusplus
}
#endif

#endif
