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
#include <string.h>
#define FOSC 7372800

#define BDIV (((FOSC / 100000) - 16) / 2 + 1)

#define TCA8418_ADDR 0x34  // 7-bit address is 0x1A, shifted left <<1 gives 0x34
#define REG_CFG         0x01
#define REG_KP_GPIO1    0x1D
#define REG_KP_GPIO2    0x1E
#define REG_KP_GPIO3    0x1F
#define TCA_WRITE_ADDR  0x68
#define TCA_READ_ADDR   0x69

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include<avr/interrupt.h>
#include <stdbool.h> 
#include "i2c.h"

volatile bool flag_tca_int = false; 

typedef enum{
    state_unlocked,
    state_locked,
    state_attempt_unlock
} State; 

char keypad[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};


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

void lcd_init(){
    _delay_ms(100);
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);

    // serial_out(0x53);    // Set brightness command
    // serial_out(0x08);    // Brightness level 8 (full)
    // serial_out();
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

void lcd_state_unlocked(){
    lcd_init();
    lcd_set_cursor(0x02);
    lcd_print("Porch  Protector");
    _delay_ms(10);
    lcd_set_cursor(0x42);
    lcd_print("*** UNLOCKED ***");
    _delay_ms(10);
    lcd_set_cursor(0x54);
    lcd_print("  PRESS # FOR LOCK");
    
}


void lcd_clear(void) {
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);    // Clear screen command
    _delay_ms(2);        // Wait for LCD to process (needs about 1.5ms)
}


//updated tca module
void tca8418_init() {
    uint8_t reset_cmd[] = { 0x1E, 0xFF }; 
    i2c_io(0x68, reset_cmd, 2, NULL, 0); // TCA software reset

    unsigned char status;
    //char temp[20]; //to read later on
    
    //writing 0x0F to KP_GPIO1 register
    uint8_t config[2];
    config[0] = REG_KP_GPIO1;
    config[1] = 0x0F; // activating row[3-0] KP_GPIO1
    status = i2c_io(0x68, config, 2, NULL, 0);

    config[0] = REG_KP_GPIO2;
    config[1] = 0x07; // activating col[2-0] KP_GPIO2
    status = i2c_io(0x68, config, 2, NULL, 0);

    status = i2c_io(0x68, (uint8_t[]){ 0x1F, 0x00 }, 2, NULL, 0); // KP_GPIO3
    status = i2c_io(0x68, (uint8_t[]){ 0x01, 0x41 }, 2, NULL, 0); // enable keypad interrupts
    status = i2c_io(0x68, (uint8_t[]){ 0x03, 0x00 }, 2, NULL, 0); // KEY_LCK_EC: clear K_LCK_EN , activate keypad scanning 

  
    // TEST CODE
    //initial I2C check to see if TCA ack
    if (status != 0) {
        lcd_set_cursor(0x00);
        lcd_print("Keypad Init Fail");
        return;
    }else{
        lcd_set_cursor(0x00);
        lcd_print("Keypad Init Success");
    }
    // Test code: reads for a key press and displays it 
    _delay_ms(10); 
    _delay_ms(3000);
    // uint8_t wbuf[1] = { 0x03 }; 
    // uint8_t rbuf[1] = { 0x00 };
    // wbuf[0] = 0x04 ; 
    // rbuf[0] = 0x00 ;
    // status = i2c_io(0x69, wbuf, 1, rbuf, 1); //0x69 address to read

    // // decode row and column
    // char number = rbuf[0] & 0x7F;    //mask out MSB
    // int row = ((int)number - 1) / 10;
    // int col = ((int)number - 1) % 10;

    // char keypad[4][3] = {
    //     {'1', '2', '3'},
    //     {'4', '5', '6'},
    //     {'7', '8', '9'},
    //     {'*', '0', '#'}
    // };

    // lcd_set_cursor(0x40);
    // lcd_print("Key Press: ");
    // lcd_print((char[]){keypad[row][col], '\0'});
}





ISR(PCINT1_vect){
    //update volatile flag: int flag 
    flag_tca_int = true; 
    
}






int main(void)
{
    //confirm this 
    sei(); //interrupt enabler
    PCMSK1 |= (1<<PCINT11);
    PCICR  |= (1<<PCIE1);
 
    serial_init(47);
    lcd_init();
    i2c_init(BDIV);
    tca8418_init();

    _delay_ms(1000);
    // FSM
    State current_state  = state_unlocked;

    //test code, check if flag is set from ISR
    lcd_set_cursor(0x40);
    if(flag_tca_int){
        lcd_print("flag TRUE");
    }
    else {
        lcd_print("flag FALSE"); 
    }

    char user_input[6] = "";
    int user_index = 0;
    char global_code[6] = "12345";


    while(1){ //state machine
        switch(current_state){
            case state_unlocked: { // case block used to allow for variable declarations
                bool flag_hashtag_received = false; 

                //lcd_state_unlocked(); //commented for testing
                lcd_set_cursor(0x54); 
                lcd_print("Pressed: ");
                while(!flag_hashtag_received){ // wait for hashtag
                    
                    if(flag_tca_int){
                        flag_tca_int = false;
                    
                        // reading int_stat to ACK receipt of int
                        uint8_t int_stat_reg = 0x02;
                        uint8_t int_status = 0;
                        i2c_io(0x69, &int_stat_reg, 1, &int_status, 1);
                    
                        // checking if key event (0x01) is the cause of the interrupt
                        if (int_status & 0x01) {
                            // reading fifo
                            uint8_t fifo_cnt_reg = 0x03;
                            uint8_t fifo_cnt = 0;
                            i2c_io(0x69, &fifo_cnt_reg, 1, &fifo_cnt, 1);
                            fifo_cnt &= 0x0F;
                            int i;
                            for (i = 0; i < fifo_cnt; i++) {
                                uint8_t fifo_reg = 0x04;
                                uint8_t key_event = 0;
                                i2c_io(0x69, &fifo_reg, 1, &key_event, 1);
                    
                                if (key_event & 0x80) {
                                    key_event &= 0x7F;
                                    int row = (key_event - 1) / 10;
                                    int col = (key_event - 1) % 10;
                                    char ch = keypad[row][col];
                                    char buf[4];
                                    sprintf(buf, "%c", ch);
                                    lcd_print(buf); //printing in real time
                                    if(ch == '*'){
                                        user_input[user_index] = '\0';
                                        if (strcmp(global_code, user_input) == 0) {
                                            lcd_print("PASS");
                                            //line to transition
                                        }else{
                                            lcd_print(" NO PASS");
                                            
                                        }
                                        lcd_set_cursor(0x54);
                                        lcd_print("IN:");
                                        char temp[10];
                                        //sprintf(temp, "%10s", user_input); 
                                        //lcd_print()
                                        lcd_print(user_input);
                                        user_input[0] = '\0';
                                        user_index = 0;            // Reset input
                                        
                                    } else {
                                        if (user_index < sizeof(user_input) - 1) {
                                            user_input[user_index++] = ch;  // Append char to string
                                        }
                                    }

                                    /*
                                    if (ch == '#'){
                                        flag_hashtag_received = true;
                                        current_state = state_locked; 
                                        break; 
                                    }
                                    */
                                }
                            }
                        }
                        //writing to INT_STAT to clear interrupt bits
                        uint8_t clear_int[] = {0x02, 0x1F};
                        i2c_io(0x68, clear_int, 2, NULL, 0);
                    }    
                }
                break;
            }
               
            
            case state_locked:
                //IO 
                //lcd locked state
                //solenoid locked state 

                //next state logic
                while(1){
                    //password array counter

                    //if int flag is set
                        //unload contents
                        //reset tca flag 
                    //fill up temp password array
                    //update lcd with unloaded contents

                    //when size is 4, check. 
                    //if match, display message (unlocking)
                    //else clear 
                    //check the password array for match 

                }
                break;
            
            case state_attempt_unlock:
                
                break; 
        }
    }
    return 0;   /* never reached */
}