#include "STM32L1xx.h" /* Microcontroller information */
const int COLUMN_MASK[] = {GPIO_BSRR_BR_4, GPIO_BSRR_BR_5, GPIO_BSRR_BR_6, GPIO_BSRR_BR_7};
const int ROW_MASK[] = {GPIO_IDR_IDR_0, GPIO_IDR_IDR_1, GPIO_IDR_IDR_2, GPIO_IDR_IDR_3};
//keypad struct
unsigned short temp;
struct {
unsigned char row;
unsigned char column;
unsigned char event;
const unsigned char row1[4]; //each row has 4 columns so 4 length array
const unsigned char row2[4];
const unsigned char row3[4];
const unsigned char row4[4];
const unsigned char* keys[];
} typedef matrix_keypad;
struct {
unsigned char first;
unsigned char second;
} typedef display;

//method declarations
void setup_timers(void);
void smallDelay(void);
void delay(void);
void setup_pins(void);
void update_leds(unsigned char value);
void setup_interupts(void);
void debug_leds(unsigned char value);

//initialize keypad with default values
matrix_keypad keypad = {
.row = ~0, //Initialize rows to MAX
.column = ~0, //Initialize columns to MAX
.event = 0,
.row1 = {1, 2, 3, 0xA},
.row2 = {4, 5, 6, 0xB},
.row3 = {7, 8, 9, 0xC},
.row4 = {0xE, 0, 0xF, 0xD},//E = *, F = #
.keys = {keypad.row1, keypad.row2, keypad.row3, keypad.row4},
};
display counter = {
.first = 0,
.second = 0,
}; 
// Delay function - do nothing for about 1 second
void delay () {
    int i,j;
    for (i=0; i<20; i++) {
        for (j=0; j<20000; j++) {
            asm("nop");
        }
    }
}

void smallDelay () {
    int i;
    for (i=0; i<4; i++) {
            asm("nop");
    }
}
/* Initialize GPIO pins used in the program.
* PA1 -> input to detect keypress
* PB[0,3] -> input to detect row of keypress
* PB[4,7] -> output to short column lines of keypad
* PC[0,3] -> output count/keypress*/
void setup_pins () {
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
        CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER1); // set to input mode

        /*Configure PB[0,3] as keypad row lines, input*/
        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
        CLEAR_BIT(GPIOB->MODER, 0x000000FF);
        /*This enables the pullup resistor so we can read the row lines*/
        MODIFY_REG(GPIOB->PUPDR, 0x000000FF, 0x00000055);
        /*Configure PB[4,7] as keypad column lines, output*/
        MODIFY_REG(GPIOB->MODER, 0x0000FF00, 0x00005500);


        // Configure PC[0,7] as output pins to drive LEDs 
        SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) 
        CLEAR_BIT(GPIOC->MODER, 0x0000FFFF); // Clear PC[0,3] mode bits 
        SET_BIT(GPIOC->MODER, 0x00005555); // General purpose output mode


}
void setup_timers() {
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); //enable clock source
    TIM10->ARR = 0x333; //set auto reload. assumes 2MHz
    TIM10->PSC=0xFF; //set prescale. assumes 2MHz
    SET_BIT(TIM10->DIER, TIM_DIER_UIE); //enable interupts
} 
void update_leds(unsigned char value) {
    unsigned int debug = (~value & 0xF) << 16;
    SET_BIT(GPIOC->BSRR, debug); // turn off LEDs not used
    SET_BIT(GPIOC->BSRR, value); // turn on LEDs
}
// Setup external inturupt on PA1 to detect a keypress
void setup_interupts() {
    // Configure GPIO A1 as external inturupt 1
    SYSCFG->EXTICR[0] &= 0xFF0F; //clear EXTL1 field
    SYSCFG->EXTICR[0] |= 0x0002; //set to PA1
    SET_BIT(EXTI->IMR, 0x0002);//enable bit
    SET_BIT(EXTI->FTSR, 0x0002);//falling edge
    NVIC_EnableIRQ(EXTI1_IRQn);
		NVIC_EnableIRQ(TIM10_IRQn); 
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
		NVIC_ClearPendingIRQ(TIM10_IRQn); 
}
//MAIN
int main() {
    setup_pins();
    setup_interupts();
    setup_timers();
    
    GPIOB->BSRR = 0x00F00000; //Ground all columns
    __enable_irq();//enable interrupts
    unsigned char running;
    while(1) {
        running = READ_BIT(TIM10->CR1, TIM_CR1_CEN);//determines if counting is happening
        if (keypad.event == 0 && running) {
            CLEAR_BIT(TIM10->CR1, TIM_CR1_CEN); //toggle counting
            keypad.event = ~0;
        }
        else if (keypad.event == 0 && !running) {
            SET_BIT(TIM10->CR1, TIM_CR1_CEN); //toggle counting
            keypad.event = ~0;
        } 
        else if (!running && keypad.event == 1) {
            counter.first = 0;
            counter.second = 0;
            GPIOC->BSRR = 0xFF0000;
            keypad.event = ~0;
        }
    }
}
void EXTI1_IRQHandler() {

SET_BIT(EXTI->PR, EXTI_PR_PR1);

    for (keypad.column=0;keypad.column<4;keypad.column++) {
        //unsigned int DEBUG_BSRR = 0;
        SET_BIT(GPIOB->BSRR, 0x000000F0);
        SET_BIT(GPIOB->BSRR, COLUMN_MASK[keypad.column]);
        //GPIOB->BSRR = DEBUG_BSRR;
        for (keypad.row=0;keypad.row<4;keypad.row++) {
            smallDelay();
            //unsigned short DEBUG_IDR = GPIOB->IDR;
            temp = READ_BIT(GPIOB->IDR, ROW_MASK[keypad.row]);
            if (!temp) {
                keypad.event = keypad.keys[keypad.row][keypad.column]; 
                SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
                return;
            }
        }
    }

    SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
} 
void TIM10_IRQHandler() {
    CLEAR_BIT(TIM10->SR, TIM_SR_UIF);
    counter.second = (counter.second + 1) % 10;
    GPIOC->BSRR = 0xF0000;
    SET_BIT(GPIOC->BSRR, counter.second & 0xF); //turn on decimal


    if (counter.second == 0) {
        counter.first = (counter.first + 1) % 10;
        GPIOC->BSRR = 0xF00000;
        SET_BIT(GPIOC->BSRR, counter.first << 4);
    }

    NVIC_ClearPendingIRQ(TIM10_IRQn);
}