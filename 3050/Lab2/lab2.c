/*====================================================*/
/* Victor P. Nelson                                               
*/ 
/* ELEC 3040/3050 -
 Lab 1, Program 1                              
*/ 
/* Toggle LED1 while button pressed, with short delay inserted   
*/ 
/*====================================================*/
#include "STM32L1xx.h"
      /* Microcontroller information */
/* Define global variables
 */
int toggles;
static int count; 
/* number of times LED state toggled */
/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program  */
/*       PA0 = push button                               */
/*       PC8 = blue LED, PC9 = green LED    */
/*---------------------------------------------------*/
void  PinSetup () { 

    RCC->AHBENR |=  0x04;                  /* Enable GPIOC clock (bit 2) */
    //New pins
    GPIOA->MODER    &= ~0x0000003C;  //sets PA1 and PA2 to '11' effectively as inputs  | PA1= start/stop | PA2 = directions
    GPIOC->MODER    |=  0x00000055; // 5= '0101' so PC3,PC2,PC1,PC0 = '01' which are outputs  
} 

void counting(){
    switch(count){
        case 0 :
        GPIOC->ODR = 0x00000000;break;
        case 1 :
        GPIOC->ODR = 0x00000001;break;
        case 2 :
        GPIOC->ODR = 0x00000002;break;
        case 3 :
        GPIOC->ODR = 0x00000003;break;
        case 4 :
        GPIOC->ODR = 0x00000004;break;
        case 5 :
        GPIOC->ODR = 0x00000005;break;
        case 6 :
        GPIOC->ODR = 0x00000006;break;
        case 7 :
        GPIOC->ODR = 0x00000007;break;
        case 8 :
        GPIOC->ODR = 0x00000008;break;
        case 9 :
        GPIOC->ODR = 0x00000009;break;
        default:
        break;
    }
}

void delay () {
   int i,j,n;
   for (i=0; i<20; i++) {       
 //outer loop 
         for (j=0; j<2000; j++) {   
//inner loop 
n = j;
//dummy operation for single-step test
         }                            
//do nothing         
    } 
} 

int main(void) { 
 
  PinSetup();
    unsigned char startStop;
    unsigned char direction;

    startStop = 0;
    direction = 0;

  while (1) {       

    while (startStop == 0) {          //Wait for SW1 = 1 
        startStop = GPIOA->IDR & 0x02;    //Read GPIOA and mask all but bit 1
    } 
    
    direction = GPIOA->IDR & 0x04;
    if(direction == 0){ //Read GPIOA and mask all but bit 2  | check to see the direction switch
        //count is down
        if(count == 0)
            count = 9;
        else
            count = count - 1;
        
    }
    else{
        if(count == 9)
        count = 0;
        else
            count = count + 1;
    } 
    counting();

    delay();                
            
    startStop = GPIOA->IDR & 0x02; // check to see if the startStop switch has been turned off 

  } 
} 