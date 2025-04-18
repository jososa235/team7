/*************************************************************
*       at328-0.c - Demonstrate simple I/O functions of ATmega328
*
*       Program loops turning PC0 on and off as fast as possible.
*
* The program should generate code in the loop consisting of
*   LOOP:   SBI  PORTC,0        (2 cycles)
*           CBI  PORTC,0        (2 cycles)
*           RJMP LOOP           (2 cycles)
*
* PC0 will be low for 4 / XTAL freq
* PC0 will be high for 2 / XTAL freq
* A 9.8304MHz clock gives a loop period of about 600 nanoseconds.
*
* Revision History
* Date     Author      Description
* 09/14/12 A. Weber    Initial Release
* 11/18/13 A. Weber    Renamed for ATmega328P
*************************************************************/
#define FOSC 7372800
#define BDIV (((FOSC / 100000) - 16) / 2 + 1)
#define TCA8418_RX_ADDR 0x69
#define TCA8418_TX_ADDR 0x68
#define TCA8418_ADDR 0x34

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c.h"


void serial_out(char ch) {
    while ((UCSR0A & (1<<UDRE0)) == 0);
    UDR0 = ch; 
}


void serial_init(unsigned short ubrr) {
    UBRR0 = ubrr;
    UCSR0B |= (1 << TXEN0); 
    UCSR0B |= (1 << RXEN0); 
    UCSR0C = (3 << UCSZ00);

}

void display_init(){
    serial_out(0xFE);    // Command prefix
    serial_out(0x53);    // Set brightness command
    serial_out(0x08);    // Brightness level 8 (full)
}

void lcd_command(uint8_t cmd) {
    serial_out(0xFE);   // Command prefix
    serial_out(cmd);    // Command byte
}
void lcd_set_cursor(uint8_t pos) {
    lcd_command(0x45);  // Set cursor command
    serial_out(pos);    // Position byte
}
void lcd_print(const char* str) {
    while (*str) {
        serial_out(*str++);
    }
}

void state_unlocked(){
    lcd_set_cursor(2);
    lcd_print("Porch  Protector");
    lcd_set_cursor(42);
    lcd_print("*** UNLOCKED ***");
    
}
void lcd_clear(void) {
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);    // Clear screen command
    _delay_ms(2);        // Wait for LCD to process (needs about 1.5ms)
}

void i2c_scan_bus(void) {
    char buf[10];
    uint8_t addr;
    uint8_t dummy_reg = 0x00;
    uint8_t dummy_read;

    for (addr = 0x03; addr <= 0x77; addr++) {
        // Convert 7-bit address to 8-bit write address
        uint8_t write_addr = (addr << 1);
        uint8_t status = i2c_io(write_addr, &dummy_reg, 1, &dummy_read, 0);

        if (status == 0) {
            lcd_set_cursor(0x00);  // top-left corner
            sprintf(buf, "Found: %02X", addr);
            lcd_print(buf);
            _delay_ms(1000);
        }
    }

    lcd_set_cursor(0x40);  // second line
    lcd_print("Scan Done");
}

int main(void)
{
    
    serial_init(47);
    display_init(); 
    _delay_ms(100);
    lcd_clear();
    state_unlocked();
    i2c_init(BDIV);


    i2c_scan_bus();


    return 0;   /* never reached */
}