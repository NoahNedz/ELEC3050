#include "STM32L1xx.h" 
    
    void delay(void);
    void count(unsigned char* counter, unsigned char direction);
    void setup_pins(void);
    void update_leds(void);
    
    unsigned char count_one = 0;
    unsigned char count_two = 0;

    //Initialize GPIO pins															

    void setup_pins () {
		/* Configure PA1 and PA2 as input pin to read push button */
		SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
		CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER1 | GPIO_MODER_MODER2)); // set to input mode
		/* Configure PC[0,7] as output pins to drive LEDs */
		SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2) */
		CLEAR_BIT(GPIOC->MODER, 0x0000FFFF); /* Clear PC[0,7] mode bits */
		SET_BIT(GPIOC->MODER, 0x00005555); /* General purpose output mode*/
    }
		

    // Delay function waits .5s																	

    void delay () {
		int i,j;
		for (i=0; i<10; i++) { //outer loop
			for (j=0; j<20000; j++) { //inner loop	
			} 
		}
    }
		
    // Incriment or deincriment toggle mod 10 up/down	   

    void count(unsigned char* counter, unsigned char direction) {
		if (direction == 0) 
			*counter = (*counter + 1) % 10;
		else 
			*counter = (*counter + (10 - 1)) % 10;
    }

    // Update the LEDS 																	 

    void update_leds() {
		unsigned short leds = (count_two & 0x0F) << 4; // PC[7:4]
		leds += (count_one & 0x0F); // PC[3:0]
		SET_BIT(GPIOC->BSRR, (~leds) << 16); // turn off LEDs
		SET_BIT(GPIOC->BSRR, leds); // turn on LEDs
    }
		
    int main() {
		unsigned char status = 0; // start/stop
		unsigned char direction = 0; // inital direction
		
		setup_pins();
		
		while(1) {
			status = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR_2); //PA1
			direction = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR_1); //PA2;
			if (status) {
				count(&count_one, direction);
				count(&count_two, (~direction) & GPIO_IDR_IDR_1);
				update_leds();
				delay();
			}
		}
    }
    
    
    

    
    
