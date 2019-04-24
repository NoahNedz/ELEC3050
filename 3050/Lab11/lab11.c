#include "STM32L1xx.h" /* Microcontroller information */
const int COLUMN_MASK[] = {GPIO_BSRR_BR_4, GPIO_BSRR_BR_5, GPIO_BSRR_BR_6, GPIO_BSRR_BR_7};
const int ROW_MASK[] = {GPIO_IDR_IDR_0, GPIO_IDR_IDR_1, GPIO_IDR_IDR_2, GPIO_IDR_IDR_3};
//keypad struct
unsigned short temp;
struct {
    unsigned char row;
    unsigned char column;
    unsigned char event;
    unsigned char value;
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

//method declarations and global variables
void setup_timers(void);
void setup_timersSmallCount(void);
void smallDelay(void);
void delay(void);
void delay2(void);
void setup_pins(void);
void update_leds(unsigned char value);
void setup_interupts(void);
void debug_leds(unsigned char value);
void speed_controller();
void setup_speed_ctr();
double period;
double period2;
double up1;
double up2;
double sum,voltage = 0;
int inputOld = 0;
int adcArrayIndex = 0; 
double adcArray = 0;
double expectedAdc[11] = {0,0.6,0.84,1.05,1.3,1.47,1.9,2.09,2.43,2.77,2.99};

//PWM vars
int speedSelect = 0;
int speedSettter = 0;
int PWMset = 0;
int stopCounter = 0;

//ADC recording vars
int AdcCount = 0;
double newAdc[2000];
int AdcStart = 0;

//PID vars
double PID();
double preError = 0;
double error = 0;
double PIDout = 0;

double integral = 0;
double derivative = 0;

double Pout = 0;
double Iout = 0;
double Dout = 0;

unsigned char running = 0;
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
.second = 0
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

void delay2 () {
    int i,j;
    for (i=0; i<5; i++) {
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
* PA1 -> input to detect key-press
* PB[0,3] -> input to detect row of key-press
* PB[4,7] -> output to short column lines of keypad
* PC[0,3] -> output count/key-press*/
void setup_pins () {
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock (bit 0)
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER1); // set to input mode

	/*Configure PB[0,3] as keypad row lines, input*/
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
	CLEAR_BIT(GPIOB->MODER, 0x000000FF);
	/*This enables the pull-up resistor so we can read the row lines*/
	MODIFY_REG(GPIOB->PUPDR, 0x000000FF, 0x00000055);
	/*Configure PB[4,7] as keypad column lines, output*/
	MODIFY_REG(GPIOB->MODER, 0x0000FF00, 0x00005500);


	// Configure PC[0,7] as output pins to drive LEDs
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN); // Enable GPIOC clock (bit 2)
	CLEAR_BIT(GPIOC->MODER, 0x0000FFFF); // Clear PC[0,3] mode bits
	SET_BIT(GPIOC->MODER, 0x00005555); // General purpose output mode

	// Clear and Change PA6 to alternative function mode
	GPIOA->MODER &= ~0x00003000;
	GPIOA->MODER |= 0x00002000;
	// Change the alternative function to be the CC
	GPIOA->AFR[0] &= ~0x0F000000;
	GPIOA->AFR[0] |= 0x03000000;// PA6 = AF3?
	
	MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL6, 0x03000000);
}
void setup_timers() {
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); //enable clock source
    RCC->CR |= RCC_CR_HSION;             
    // Turn on 16MHz HSI oscillator
    while((RCC->CR & RCC_CR_HSIRDY) == 0);   // Wait until HSI ready
    RCC->CFGR |= RCC_CFGR_SW_HSI;  
    // Select HSI as system clock
	
	TIM10->PSC=(1); //set pre-scale. assumes 2MHz
	TIM10->ARR = 799; //set auto reload. //10kHz
	TIM10->CCR1 = 10; //Set compare value
	TIM10->CNT = 0; //this gets compared to ccr1

	MODIFY_REG(TIM10->CCMR1, TIM_CCMR1_CC1S, 0x0000); // Capture compare 1 select to to output mode
	MODIFY_REG(TIM10->CCMR1, TIM_CCMR1_OC1M, 0x0060); // Active to inactive

	SET_BIT(TIM10->CCER, TIM_CCER_CC1E); // drive output pin
	SET_BIT(TIM10->CR1, TIM_CR1_CEN); //enable counting

	//stopwatch timer
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
	RCC->CR |= RCC_CR_HSION;
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	TIM4->PSC=(1599); 
	TIM4->ARR = 999; //1second
		
    SET_BIT(TIM4->DIER, TIM_DIER_UIE); //enable interrupts
	TIM4->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM4_IRQn);
}
void setup_timersSmallCount() {
	RCC->APB2ENR |= 0x00000004;
    RCC->CR |= RCC_CR_HSION;             
	RCC->CFGR |= RCC_CFGR_SW_HSI;  
	// Select HSI as system clock
    TIM9->PSC=(161); 
	TIM9->ARR = 999; //set auto reload. 100Hz
		
    SET_BIT(TIM9->DIER, TIM_DIER_UIE); //enable interrupts
	TIM9->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM9_IRQn);
} 




void update_leds(unsigned char value) {
    unsigned int leds = (~value & 0xF) << 16;
    SET_BIT(GPIOC->BSRR, leds); // turn off LEDs not used
    SET_BIT(GPIOC->BSRR, value); // turn on LEDs
}
// Setup external interrupt on PA1 to detect a key-press
void setup_interupts() {
    // Configure GPIO A1 as external inturupt 1
    SYSCFG->EXTICR[0] &= 0xFF0F; //clear EXTL1 field
    SYSCFG->EXTICR[0] |= 0x0002; //set to PA1
    SET_BIT(EXTI->IMR, 0x0002);//enable bit
    SET_BIT(EXTI->FTSR, 0x0002);//falling edge
    NVIC_EnableIRQ(EXTI1_IRQn);
	//NVIC_EnableIRQ(TIM10_IRQn);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
	//NVIC_ClearPendingIRQ(TIM10_IRQn);
}
void setup_speed_ctr() {
	/* Change PA7 to analog mode */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, (0xFFFFFFFF & GPIO_MODER_MODER7));
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); // Enable GPIOA clock
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN); //enable clock source
	SET_BIT(ADC1->CR2, ADC_CR2_ADON); // Turn on converter
	SET_BIT(ADC1->CR2, ADC_CR2_CONT); // Enable continuous conversion
	ADC1->SMPR3 &= ~ADC_SMPR3_SMP7;
	ADC1->SMPR3 |= 0x00300000; //sample to 24
	MODIFY_REG(ADC1->SQR5, ADC_SQR5_SQ1, 0x7); // Use PA7 for sampling
}

//MAIN
int main() {    
	setup_pins();
	setup_interupts();
	setup_timers();
	setup_timersSmallCount();
	//speed_controller();//timer for motor spped
	setup_speed_ctr();//amplitude sensing
	
    
    GPIOB->BSRR = 0x00F00000; //Ground all columns
    __enable_irq();//enable interrupts
    CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN); //stop counting 
    while(1) {
		running = READ_BIT(TIM4->CR1, TIM_CR1_CEN);//determines if counting is happening
		if(!running && keypad.event == 0xE){ SET_BIT(TIM4->CR1, TIM_CR1_CEN);stopCounter = 1;keypad.event = 0;}
		else if(!stopCounter && running && keypad.event == 0xE) { CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);stopCounter = 0;keypad.event = 0;}
		else if(!stopCounter && !running && keypad.event == 0x0F){ counter.first = 0;counter.second = 0;GPIOC->BSRR = 0xFF0000;CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);stopCounter = 1;keypad.event = 0;}
		stopCounter = 0;
		
		while ((ADC1->SR & ADC_SR_ADONS) == 0); // Wait for converter to power on		
	};
}


void speed_controller() {
    /* Change PA7 to alternative function mode */
	GPIOA->MODER &= ~0x0000C000; //[15:14] to 10 for PA7 = Alt. Function
	GPIOA->MODER |= 0x00008000;
    
    /* Change the alternate function of PA7 to be the CC*/
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL7, 0x30000000);
    
    /*This enables the pull-up resistor so we can read input capture*/
    //MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR7, 0x01);
	MODIFY_REG(GPIOA->PUPDR, 0x000000FF, 0x00000055);
    
    /* Timer 11 */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); //enable clock source

    /* CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER) */
    
    CLEAR_BIT(TIM11->CCER, TIM_CCER_CC1E); // Set to falling edge
        
    MODIFY_REG(TIM11->CCMR1, TIM_CCMR1_CC1S, 0x01); // Capture compair select input
    
    TIM11->ARR = 0xFFFF; // value > max period to prevent update event before capture
        
    TIM11->PSC = 0x00; //set pre-scale.

    /* Enabling Timer Interrupts */
    SET_BIT(TIM11->DIER, TIM_DIER_CC1IE); // enable capture compare interrupts
    CLEAR_BIT(TIM11->SR,TIM_SR_CC1IF); // clear capture/compare 1 interrupt flag
    /* CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER) */
    SET_BIT(TIM11->CCER, TIM_CCER_CC1E); // drive output pin
    SET_BIT(TIM11->CR1, TIM_CR1_CEN); //enable counting
        
	NVIC_ClearPendingIRQ(TIM11_IRQn); // Clear Pending Status of Timer IRQ
    NVIC_EnableIRQ(TIM11_IRQn);
}


void TIM11_IRQHandler() {
    CLEAR_BIT(TIM11->SR, TIM_SR_CC1IF);
    period = (TIM11->CCR1); // Because we use the high speed clock
	period2 = (period / 16000000.0);
	TIM11->CNT = 0;
    NVIC_ClearPendingIRQ(TIM11_IRQn);
}

void TIM9_IRQHandler() {
	if(speedSelect != 0){
		SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //software trigger
		sum = (ADC1->DR);
		adcArray = voltage = (sum*3)/(4096); //12bit = 4096,voltage of 3 reference
		PID();
		
		AdcCount++;
		if(AdcCount <= 2000 && AdcStart){newAdc[AdcCount] = adcArray;}
		if(AdcCount >1999){AdcStart = 0;}
	}		
	CLEAR_BIT(TIM9->SR, TIM_SR_UIF);//update interrupt flag
	TIM9->CNT = 0;
    NVIC_ClearPendingIRQ(TIM9_IRQn);
}

void TIM4_IRQHandler() {
	CLEAR_BIT(TIM4->SR, TIM_SR_UIF);
	counter.second = (counter.second + 1) % 10;
	GPIOC->BSRR = 0xF0000;
	SET_BIT(GPIOC->BSRR, counter.second & 0xF); //turn on decimal
	if (counter.second == 0) {
		counter.first = (counter.first + 1) % 10;
		GPIOC->BSRR = 0xF00000;
		SET_BIT(GPIOC->BSRR, counter.first << 4);
	}

    NVIC_ClearPendingIRQ(TIM4_IRQn);
}


void EXTI1_IRQHandler() {

    for (keypad.column=0;keypad.column<4;keypad.column++) {
        
        SET_BIT(GPIOB->BSRR, 0x000000F0);
        SET_BIT(GPIOB->BSRR, COLUMN_MASK[keypad.column]);
        
        for (keypad.row=0;keypad.row<4;keypad.row++) {
            smallDelay();
            temp = READ_BIT(GPIOB->IDR, ROW_MASK[keypad.row]);
            if (!temp) {		
                keypad.value = keypad.keys[keypad.row][keypad.column];
                NVIC_ClearPendingIRQ(EXTI1_IRQn);
                SET_BIT(EXTI->PR, EXTI_PR_PR1);
                //update_leds(keypad.value);
				inputOld = input1;
				input1 = keypad.value;
				input2 = inputOld;
				keypad.event = input1;
				
				if(input1 != input2){input2 = inputOld;}
								
                if(keypad.value == 0x00){PWMset = 0;TIM10->CCR1 = 0;speedSelect = 0;}
									
					else if(keypad.value == 0x01){PWMset = 280;TIM10->CCR1 = 280;speedSelect = 1;}//35%
					else if(keypad.value == 0x02){PWMset = 320;TIM10->CCR1 = 320;speedSelect = 2;}//40%
					else if(keypad.value == 0x03){PWMset = 360;TIM10->CCR1 = 360;speedSelect = 3;}//45%
					else if(keypad.value == 0x04){PWMset = 400;TIM10->CCR1 = 400;AdcStart = 1;speedSelect = 4;}//50%
					else if(keypad.value == 0x05){PWMset = 440;TIM10->CCR1 = 440;AdcStart = 1;speedSelect = 5;}//55%
					else if(keypad.value == 0x06){PWMset = 520;TIM10->CCR1 = 520;speedSelect = 6;}//65%
					else if(keypad.value == 0x07){PWMset = 560;TIM10->CCR1 = 560;speedSelect = 7;}//70%
					else if(keypad.value == 0x08){PWMset = 640;TIM10->CCR1 = 640;	speedSelect = 8;}//80%
					else if(keypad.value == 0x09){PWMset = 720;TIM10->CCR1 = 720;speedSelect = 9;}//90%
					else if(keypad.value == 0x0A){PWMset = 800;TIM10->CCR1 = 800;	speedSelect = 10;}//100%
					else if(keypad.value == 0x00){TIM10->CCR1 = 0; speedSelect = 0;}//0%			
					SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns
                return;
            }				
        }
    }
		
	smallDelay();
    SET_BIT(GPIOB->BSRR, 0x00F00000); //Ground all columns 
}

double PID(){
	double kp = 1;
	double kd = 0.2;
	double ki = 1;
	error = expectedAdc[speedSelect] - adcArray;
	Pout = kp*error;
	Iout = error * ki * .001;
	derivative = (error - preError) / .01;
	Dout = kd*derivative;
	PIDout = Dout+Iout + Pout;
	PWMset += PWMset*(PWMset/error*10);
	TIM10->CCR1 = PWMset;

	//else if(PIDout < 0){
		//TIM10->CCR1 = 0;//0%
		//speedSelect = 0;
	//}
	
	if(expectedAdc[speedSelect] == 0){
		TIM10->CCR1 = 0;
	}
	preError = error;
	if(adcArray+0.01 < expectedAdc[speedSelect]){
		PWMset += 1;
		TIM10->CCR1 = PWMset;
		
	}
	else if(adcArray-0.04 > expectedAdc[speedSelect]){
		PWMset -= 1;
		TIM10->CCR1 = PWMset;
	}
	
	return 0;
}
