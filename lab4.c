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
#include <stdio.h>
#include <stdbool.h>

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
volatile bool flag_prev_motion_lvl = false; 

typedef enum{
    state_unlocked,
    state_locked,
    state_master_reset,
    state_reset_code
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
    _delay_ms(10);
    serial_out(0xFE);    // Command prefix
    serial_out(0x41);
    _delay_ms(10);
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);
    _delay_ms(10);
    serial_out(0xFE);
    serial_out(0x53);    // Set brightness command
    serial_out(0x06);    // Brightness level 8 (full)
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
    lcd_print("  PRESS # FOR LOCK, * TO RESET ");
    
}

void lcd_state_locked(){
    lcd_init();
    lcd_set_cursor(0x02);
    lcd_print("Porch Protector");
    _delay_ms(10);
    lcd_set_cursor(0x42);
    lcd_print("*** LOCKED ***");
    _delay_ms(10);
    lcd_set_cursor(0x14);
    lcd_print("CODE AND * TO UNLOCK");
}

void lcd_state_reset(){
    lcd_init();
    lcd_set_cursor(0x02);
    lcd_print("Porch Protector");
    _delay_ms(10);
    lcd_set_cursor(0x42);
    lcd_print("*INPUT MASTER CODE TO CONT.*");
    _delay_ms(10);
    lcd_set_cursor(0x54);
    lcd_print("CODE & * OR # TO RETURN");
}

void lcd_state_reset_code(){
    lcd_init();
    lcd_set_cursor(0x02);
    lcd_print("Porch Protector");
    _delay_ms(10);
    lcd_set_cursor(0x42);
    lcd_print("INPUT NEW CODE + *, # TO CANCELL");
    _delay_ms(10);
    //lcd_set_cursor(0x54);
    //lcd_print("NEW CODE:");
}


void lcd_clear(void) {
    serial_out(0xFE);    // Command prefix
    serial_out(0x51);    // Clear screen command
    _delay_ms(2);        // Wait for LCD to process (needs about 1.5ms)
}


//potential code cleanup code
void read_input(char input[], bool *hash_flag, bool *star_flag){
    bool flag_passcode_correct = false;
    int user_index = 0;
    //char user_input[10];
    while(!flag_passcode_correct){ // wait for hashtag
        if(flag_tca_int){
            flag_tca_int = false;
            //lcd_set_cursor(0x14);
            //lcd_print("In flag loop");
            //lcd_set_cursor(0x5C); //setting cursor back for passcode entry display
        
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

                        if(ch == '#'){
                            //an idea can just return '#' having been pressed aswell same for '*' and return user_input for that 

                            //all states just use # as a flag so no more logic needed
                            *hash_flag = true; //changes flag pointer outside this functoin logic
                            break;

                        }else if(ch == '*'){ //action * button
                            input[user_index] = '\0';
                            //lcd output is code matches
                            
                            *star_flag = true;

                            //lcd_set_cursor(0x54);
                            //lcd_print("IN:");
                            //char temp[10];
                            //sprintf(temp, "%10s", user_input); 
                            //lcd_print();
                            //lcd_print(user_input);       // Reset input
                            //flag_passcode_correct = false;
                            
                        } else {
                            if (user_index < sizeof(input) - 1) {
                                input[user_index++] = ch;  // Append char to string
                            }
                        }
                    }
                }
            }
            //writing to INT_STAT to clear interrupt bits
            uint8_t clear_int[] = {0x02, 0x1F};
            i2c_io(0x68, clear_int, 2, NULL, 0);
        }    
    }
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


void motion_check(){
    lcd_set_cursor(0x00);
    lcd_print("No Motion");
    _delay_ms(500);
    while(1){
        uint8_t pd4_value = (PIND & (1 << PD4)) != 0;

        if(pd4_value){
            lcd_set_cursor(0x00);
            lcd_print("Motion Detected");
            break;
        }
    }
    
}


ISR(PCINT1_vect){
    //update volatile flag: int flag 
    flag_tca_int = true; 
    
}

void timer1_init() {
    // Set Timer1 with 1024 prescaler
    TCCR1A = 0;                       // Normal mode
    TCCR1B = (1 << CS12) | (1 << CS10);  // Prescaler = 1024

    // Load initial timer value for 5s overflow
    TCNT1 = 0xFFFF;

    // Enable Timer1 overflow interrupt
    TIMSK1 |= (1 << TOIE1);
}

ISR(TIMER1_OVF_vect) {
    // This runs every 5 seconds
    // Place your interrupt code here (e.g., toggle LED)
    bool current_lvl = (PIND & (1<<PD4)) ? true : false; 

    if(current_lvl == flag_prev_motion_lvl && !current_lvl){
        //turns lcd off
        _delay_ms(10);
        serial_out(0xFE);
        serial_out(0x53);    // Command prefix
        serial_out(0x01);
        TCNT1 = 0xFFFF;
    }
    else {
        
        //lcd_init(); 
        _delay_ms(10);
        serial_out(0xFE);
        serial_out(0x53);    // Command prefix
        serial_out(0x06);
        TCNT1 = 3000;
    }


    
}



int main(void)
{
    
    sei(); //interrupt enabler
    PCMSK1 |= (1<<PCINT11);
    PCICR  |= (1<<PCIE1);

    //motion sensor pin setup
    DDRD &= ~(1 << PD4);
    
    //timer configuration 
    timer1_init(); 

    serial_init(47);
    lcd_init();
    i2c_init(BDIV);
    tca8418_init();

    _delay_ms(1000);
    motion_check(); //polling for motion to wakeup;
    _delay_ms(1000);
    // FSM
    State current_state  = state_locked;

    //test code, check if flag is set from ISR
    lcd_set_cursor(0x40);
    if(flag_tca_int){
        lcd_print("flag TRUE");
    }
    else {
        lcd_print("flag FALSE"); 
    }

    char user_input[10] = "";
    int user_index = 0;
    char global_code[10] = "12345";
    char master_code[10] = "54321";

    //lcd display off test code
    // _delay_ms(10);
    // lcd_command(0x53);
    // serial_out(0x01);
    // _delay_ms(10);

    while(1){ //state machine
        switch(current_state){
            case state_locked: { // case block used to allow for variable declarations
                bool flag_passcode_correct = false; 
                lcd_state_locked(); 
                lcd_set_cursor(0x54); 
                lcd_print("Pressed: ");
                while(!flag_passcode_correct){ // wait for hashtag
                    if(flag_tca_int){
                        flag_tca_int = false;
                        //lcd_set_cursor(0x14);
                        //lcd_print("In flag loop");
                        //lcd_set_cursor(0x5C); //setting cursor back for passcode entry display
                    
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
                                        //lcd output is code matches
                                        if (strcmp(global_code, user_input) == 0) {
                                            lcd_set_cursor(0x54);
                                            lcd_print("               ");
                                            lcd_set_cursor(0x54);
                                            lcd_print("PASS");
                                            flag_passcode_correct = true;
                                            current_state = state_unlocked; //state transition
                                            user_input[0] = '\0';
                                            user_index = 0;   
                                            break;
                                        }else{ //lcd output if code is incorrect
                                            lcd_set_cursor(0x54);
                                            lcd_print("               ");
                                            lcd_set_cursor(0x54);
                                            lcd_print("NO PASS");
                                            _delay_ms(500);
                                            lcd_set_cursor(0x54); 
                                            lcd_print("Pressed: ");
                                            //lcd_set_cursor(0x14);
                                            //lcd_print("         ");
                                            _delay_ms(1000);
                                            flag_tca_int = true;
                                            user_input[0] = '\0';
                                            user_index = 0;     
                                            current_state = state_locked;
                                            break;
                                        }
                                        //lcd_set_cursor(0x54);
                                        //lcd_print("IN:");
                                        //char temp[10];
                                        //sprintf(temp, "%10s", user_input); 
                                        //lcd_print();
                                        //lcd_print(user_input);       // Reset input
                                        flag_passcode_correct = false;
                                        
                                    } else {
                                        if (user_index < sizeof(user_input) - 1) {
                                            user_input[user_index++] = ch;  // Append char to string
                                        }
                                    }
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
               
            
            case state_unlocked:{
                //IO 
                //lcd locked state
                //solenoid locked state 

                //next state logic
                bool flag_hashtag_received = false;

                lcd_state_unlocked(); //commented for testing
                // lcd_set_cursor(0x00);
                // lcd_print("Current State: Unlocked");
                // lcd_set_cursor(0x54); 
                // lcd_print("Pressed: ");
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
                                    if (ch == '#'){
                                        flag_hashtag_received = true;
                                        current_state = state_locked; 
                                        break; 
                                    }
                                    if(ch == '*'){ //goes to reset screen
                                        flag_hashtag_received = true;
                                        current_state = state_master_reset;
                                        break;
                                    }
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

            case state_master_reset:{
                lcd_state_reset();
                bool flag_passcode_correct = false;
                //clearing user input character array
                user_input[0] = '\0';
                user_index = 0;  
                while(!flag_passcode_correct){ // wait for hashtag
                    
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

                                    //issue here # goes to reset code and not backwards
                                    if (ch == '#'){ //returns to unlocked state
                                        //clearing user input
                                        user_input[0] = '\0';
                                        user_index = 0;   
                                        //returning to previous state
                                        flag_passcode_correct = true;
                                        current_state = state_unlocked; 
                                        break; 
                                    } else if(ch == '*'){ //
                                        flag_passcode_correct = true;
                                        user_input[user_index] = '\0';
                                        //lcd output is code matches
                                        if (strcmp(master_code, user_input) == 0) {
                                            lcd_set_cursor(0x54);
                                            lcd_print("               ");
                                            lcd_set_cursor(0x54);
                                            lcd_print("PASS");
                                            flag_passcode_correct = true;
                                            current_state = state_reset_code; //state transition
                                            user_input[0] = '\0';
                                            user_index = 0;   
                                            break;
                                        }else{ //lcd output if code is incorrect
                                            lcd_set_cursor(0x54);
                                            lcd_print("               ");
                                            lcd_set_cursor(0x54);
                                            lcd_print("NO PASS");
                                            _delay_ms(500);
                                            lcd_set_cursor(0x54); 
                                            lcd_print("Pressed: ");
                                            //lcd_set_cursor(0x14);
                                            //lcd_print("         ");
                                            _delay_ms(1000);
                                            flag_tca_int = true;
                                            user_input[0] = '\0';
                                            user_index = 0; 
                                            //flag_tca_int = true;    
                                            current_state = state_master_reset; //stays in current state
                                            break;
                                        }
                                        
                                        break;
                                    }else {
                                        if (user_index < sizeof(user_input) - 1) {
                                            user_input[user_index++] = ch;  // Append char to string
                                        }
                                    }
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

            case state_reset_code:{
                bool flag_passcode_correct = false; 
                lcd_state_reset_code(); 
                lcd_set_cursor(0x54); 
                lcd_print("Pressed: ");
                //clearing user input character array
                user_input[0] = '\0';
                user_index = 0;  
                while(!flag_passcode_correct){ // wait for hashtag
                    if(flag_tca_int){
                        flag_tca_int = false;
                        //lcd_set_cursor(0x14);
                        //lcd_print("In flag loop");
                        //lcd_set_cursor(0x5C); //setting cursor back for passcode entry display
                    
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


                                    //state transitions are fine but keypad breaks after the state transitions with # and *
                                    if(ch == '#'){
                                        lcd_set_cursor(0x54);
                                        lcd_print("RESET CANCELLED");
                                        _delay_ms(1000);
                                        flag_passcode_correct = true;
                                        //_delay_ms(1000);
                                        current_state = state_unlocked;
                                        break;
                                    }else if(ch == '*'){
                                        user_input[user_index] = '\0';
                                        //copying code over (resetting code)
                                        memcpy(global_code, user_input, sizeof(user_input));  // copies all elements
                                        user_input[0] = '\0';
                                        user_index = 0;  
                                        //global_code[0] = '\0';
                                        lcd_set_cursor(0x54);
                                        lcd_print("               ");
                                        lcd_set_cursor(0x54);
                                        lcd_print("NEW CODE:");
                                        lcd_print(global_code);
                                        _delay_ms(1000);
                                        flag_passcode_correct = true;
                                        //_delay_ms(1000);
                                        current_state = state_unlocked;
                                        break;
                                        
                                    } else {
                                        if (user_index < sizeof(user_input) - 1) {
                                            user_input[user_index++] = ch;  // Append char to string
                                        }
                                    }
                                }
                            }
                        }
                        // Drain remaining FIFO
                        uint8_t fifo_cnt_reg = 0x03;
                        uint8_t fifo_cnt = 0;
                        i2c_io(0x69, &fifo_cnt_reg, 1, &fifo_cnt, 1);
                        fifo_cnt &= 0x0F;
                        int j;
                        for (j = 0; j < fifo_cnt; j++) {
                            uint8_t dummy = 0;
                            uint8_t fifo_reg = 0x04;
                            i2c_io(0x69, &fifo_reg, 1, &dummy, 1);
                        }

                        // Clear INT_STAT
                        uint8_t clear_int[] = {0x02, 0x1F};
                        i2c_io(0x68, clear_int, 2, NULL, 0);

                    }    
                }
                break;
            }
        }
    }
    return 0;   /* never reached */
}