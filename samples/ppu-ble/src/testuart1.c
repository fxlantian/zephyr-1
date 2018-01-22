#include <zephyr.h>
#include <misc/printk.h>
#include <uart.h>

#define UART_DEV_NAME "UART_1"

void main() 
{
    struct device *uart_dev = device_get_binding(UART_DEV_NAME);
    if(!uart_dev)
    {
        printk("Cannot get UART_1 device!\n");
        return;
    }
    printk("Find UART_1!\n");

    unsigned char ch[4] = {0x01, 0x03, 0x0C, 0x00};   

    uart_poll_out(uart_dev, ch[0]); 
    uart_poll_out(uart_dev, ch[1]); 
    uart_poll_out(uart_dev, ch[2]); 
    uart_poll_out(uart_dev, ch[3]);

    printk("Data has been sent out!\n");

    unsigned char recv_char = 0;

    while(uart_poll_in(uart_dev, &recv_char) < 0); 
    printk("Recieved data: %x\n", recv_char);
}
