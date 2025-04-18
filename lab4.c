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

#define TCA8418_ADDR 0x34  // 7-bit address is 0x1A, shifted left <<1 gives 0x34
#define REG_CFG         0x01
#define REG_KP_GPIO1    0x1D
#define REG_KP_GPIO2    0x1E
#define REG_KP_GPIO3    0x1F

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
    _delay_ms(100);

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
    _delay_ms(10);
    lcd_set_cursor(42);
    lcd_print("*** UNLOCKED ***");
    
}
void lcd_clear(void) {
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);    // Clear screen command
    _delay_ms(2);        // Wait for LCD to process (needs about 1.5ms)
}

void tca8418_init(){
    uint8_t config[2];
    config[0] = REG_KP_GPIO1;
    config[1] = 0x0F;
    i2c_io(TCA8418_ADDR, config, 2, NULL, 0);
}


int main(void)
{
    serial_init(47);
    state_unlocked();

    i2c_init(BDIV);
    tca8418_init();

    return 0;   /* never reached */
}