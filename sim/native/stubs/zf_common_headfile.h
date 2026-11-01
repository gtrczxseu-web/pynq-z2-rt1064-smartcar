#ifndef HOST_ZF_COMMON_H
#define HOST_ZF_COMMON_H
#include <stdint.h>
#include <math.h>
#define MIN(a,b) ((a)<(b)?(a):(b))
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void gpio_set_level(int pin,int level);
void pwm_set_duty(int pin,int duty);
void uart_write_buffer(int uart,const uint8_t *data,uint32_t length);
#define UART_1 1
#endif
