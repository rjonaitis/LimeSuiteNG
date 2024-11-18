#ifndef LIMEUART_H
#define LIMEUART_H

#include <linux/serial.h>
#include <linux/serial_core.h>

struct limeuart_port {
    struct uart_port port;
    struct timer_list timer;
    u32 id;
};

#endif