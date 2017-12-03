#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H

#include <libopencm3/stm32/usart.h>

#define P_USART USART1

void phex4(uint8_t c);
void phex(uint8_t c);
void phex16(uint16_t c);
void phex24(uint32_t c);
void phex32(uint32_t c);
void pent(void);

void pstr(char *s);

#endif /* SERIAL_HELPER_H */