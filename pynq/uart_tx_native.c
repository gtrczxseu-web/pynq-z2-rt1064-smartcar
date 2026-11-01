/* One complete 12-byte UARTLite command per native call.
 * Never read STATUS: its RX error bits are clear-on-read and owned by RX.
 * Caller serializes bursts with at least 3ms quiet time. At 115200 8N1 a
 * burst drains in ~1.05ms and fits the existing 16-byte TX FIFO.
 * The MCU's independent 0.8s command lease remains the loss/stall guard. */
#include <stdint.h>

#ifdef UART_TX_TEST
extern void uart_tx_test_store(volatile uint32_t *address, uint32_t value);
#define TX_STORE(address,value) uart_tx_test_store(address,value)
#else
#define TX_STORE(address,value) (*(address)=(value))
#endif

uint32_t uart_tx_abi(void) {return 0x54583132u;}
int uart_tx_write12(uintptr_t address, const uint8_t *packet, uint32_t length) {
    volatile uint32_t *tx;
    unsigned i;
    if(!address || (address&3u) || !packet || length!=12u) return -1;
    tx=(volatile uint32_t *)(address+4u);
    __sync_synchronize();
    for(i=0;i<12u;i++) TX_STORE(tx,(uint32_t)packet[i]);
    __sync_synchronize();
    return 0;
}
