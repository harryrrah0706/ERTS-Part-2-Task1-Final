/*
Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Author:      Mohd A. Zainol, Reza Nejabati
// Date:        10/10/2018
// Chip:        MSP432P401R LaunchPad Development Kit (MSP-EXP432P401R) for TI-RSLK
// File:        Part1_Interrupt_bumpsw_motor.h
// Function:    Part 1 of ERTS, uses interrupt for bump switches to control the motor

#include <stdint.h>
#include "msp.h"

// Check these if you really need to include these libraries in your code

#include "E://School/robo1/part1_group13/inc/Clock.h"
#include "E://School/robo1/part1_group13/inc/SysTick.h"
#include "E://School/robo1/part1_group13/inc/CortexM.h"
#include "E://School/robo1/part1_group13/inc/motor.h"


// Color    LED(s) Port2
// dark     ---    0
// blue     --B    0x04
// green    -G-    0x02
// sky blue -GB    0x06
// red      R--    0x01
// pink     R-B    0x05
// yellow   RG-    0x03
// white    RGB    0x07
#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define PINK      0x05
#define SKYBLUE   0x06
#define WHITE     0x07



int mode;
uint8_t status;




// Initialize Bump sensors using interrupt
// Make six from Port 4 input pins
// Activate interface pull-up
// The pins are P4.7, 4.6, 4.5, 4.3, 4.2, 4.0
void BumpEdgeTrigger_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // make in
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // pull-up
    P4->IES |= 0xED;        // falling edge event
    P4->IFG &= ~0xED;       // clear flag
    P4->IE |= 0xED;         // arm the interrupt
    // priority 2 on port4
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000;
    // enable interrupt 38 in NVIC on port4
    NVIC->ISER[1] = 0x00000040;
}



void Port2_Output(uint8_t data){
    // built-in red LED connected to P2.0
    // built-in green LED connected to P2.1
    // built-in blue LED connected to P2.2
    // write three outputs bits of P2
    P2->OUT = (P2->OUT&0xF8)|data;
}



void turn_direction(int direction, int time){
    if (direction==1){
        Port2_Output(GREEN);
        Motor_BackwardSimple(5000,20);
        Port2_Output(RED);
        Motor_StopSimple(25);    // Stop the motor on initial state
        Port2_Output(YELLOW);
        Motor_LeftSimple(5000,time);
        Motor_StopSimple(25);
    }
    if (direction==2){
        Port2_Output(GREEN);
        Motor_BackwardSimple(5000,20);
        Port2_Output(RED);
        Motor_StopSimple(25);    // Stop the motor on initial state
        Port2_Output(YELLOW);
        Motor_RightSimple(5000,time);
        Motor_StopSimple(25);
    }
}



// Uses P4IV IRQ handler to solve critical section/race
void PORT4_IRQHandler(void){

    uint8_t status;

    // The movement for coloured LED
    // WHITE:   Forward
    // BLUE:    Turn right
    // YELLOW:  Turn left
    // GREEN:   Backward

      status = P4->IV;      // 2*(n+1) where n is highest priority

      if (mode==1){
          Port2_Output(RED);
          P2->DIR &= ~0xC0;
          P2->OUT &= ~0xC0;
      }

      if (mode==3){
          switch(status){
            case 0x02:
                turn_direction(1,10);
              break;

            case 0x06: // Bump switch 2
                turn_direction(1,15);
                break;

            case 0x08: // Bump switch 3
                turn_direction(1,20);
                break;

            case 0x0C: // Bump switch 4
                turn_direction(2,20);
              break;

            case 0x0E: // Bump switch 5
                turn_direction(2,15);
              break;

            case 0x10: // Bump switch 6
                turn_direction(2,10);
              break;

            case 0xED: // none of the switches are pressed
              break;
          }
          Port2_Output(WHITE);
      }

      P4->IFG &= ~0xED; // clear flag
}



// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read_Input(void){
  return (P4->IN&0xED); // read P4.7, 4.6, 4.5, 4.3, 4.2, 4.0 inputs
}



// Function: checkbumpswitch(uint8_t status)
// Description: this is an alternative way that you can use, 
//              in which it uses polling method that comes from main function.
//              However it is important to note that:
//              1) the polling method is only useful for small program
//              2) the input mask in switch case (for polling method) is DIFFERENT from the 
//                 Nested Vectored Interrupt Controller (NVIC) which used in interrupt method.
void auto_checkbumpswitch(uint8_t status){
    switch(status){
      //case 0x02: // Bump switch 1 (for interrupt vector)
        case 0x6D: // Bump 1
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;
        break;
      //case 0x06: // Bump switch 2 (for interrupt vector)
        case 0xAD: // Bump 2
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;

        break;
      //case 0x08: // Bump switch 3 (for interrupt vector)
        case 0xCD: // Bump 3
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;

        break;
      //case 0x0C: // Bump switch 4 (for interrupt vector)
        case 0xE5: // Bump 4
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;

        break;
      //case 0x0E: // Bump switch 5 (for interrupt vector)
        case 0xE9: // Bump 5
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;

        break;
      //case 0x10: // Bump switch 6 (for interrupt vector)
        case 0xEC: // Bump 6
            Port2_Output(RED);
            P2->DIR &= ~0xC0;
            P2->OUT &= ~0xC0;

        break;
      case 0xED: // neither switch pressed

        break;
    }
}



void free_checkbumpswitch(uint8_t status){
    switch(status){
      case 0x02:
          turn_direction(2,10);
        break;

      case 0x06: // Bump switch 2
          turn_direction(2,15);
          break;

      case 0x08: // Bump switch 3
          turn_direction(2,20);
          break;

      case 0x0C: // Bump switch 4
          turn_direction(1,20);
        break;

      case 0x0E: // Bump switch 5
          turn_direction(1,15);
        break;

      case 0x10: // Bump switch 6
          turn_direction(1,10);
        break;

      case 0xED: // none of the switches are pressed
        break;
    }
    Port2_Output(WHITE);
}




void Port1_Init(void){
  P1->SEL0 &= ~0x01;
  P1->SEL1 &= ~0x01;        // configure P1.0 as GPIO
  P1->DIR |= 0x01;          //make P1.0 out, the built-in LED1
}



void Port2_Init(void){
    P2->SEL0 &= ~0xC7;
    P2->SEL1 &= ~0xC7;        // configure P2.2 P2.1 P2.0, and P2.6 P2.7 as GPIO
    P2->DIR |= 0x07;          // make P2.2-P2.0 out
    P2->DS |= 0x07;           // activate increased drive strength
    P2->OUT &= ~0x07;         // all LEDs off
    P2->DIR |= 0xC0;          // Direction of the motor
}



void Switch_Init(void){
    // negative logic built-in Button 1 connected to P1.1
    // negative logic built-in Button 2 connected to P1.4
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;      // configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;       // make P1.4 and P1.1 in
    P1->REN |= 0x12;        // enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;        // P1.4 and P1.1 are pull-up
}



// bit-banded addresses, positive logic
#define SW1IN ((*((volatile uint8_t *)(0x42098004)))^1)
#define SW2IN ((*((volatile uint8_t *)(0x42098010)))^1) // input: switch SW2
#define REDLED (*((volatile uint8_t *)(0x42098040)))    // output: red LED



void auto_polling(void){
    while(1){
        status = Bump_Read_Input();
        if (status == 0x6D || status == 0xAD || status == 0xCD || status == 0xE5 || status == 0xE9 || status == 0xEC) {
            auto_checkbumpswitch(status);
        }
        Motor_ForwardSimple(5000,50);
        Motor_RightSimple(5000,30);
        Motor_ForwardSimple(5000,50);
        Motor_LeftSimple(5000,30);
    }
}



void free_polling(void){
    while(1){
        status = Bump_Read_Input();
        if (status == 0x6D || status == 0xAD || status == 0xCD || status == 0xE5 || status == 0xE9 || status == 0xEC) {
            free_checkbumpswitch(status);
        }
        Port2_Output(WHITE);
        Motor_ForwardSimple(5000,100);
    }
}



void auto_interrupt(void){
    Port2_Output(WHITE);
    while(1){
        Motor_ForwardSimple(5000,50);       // Moving in predefined route.
        Motor_RightSimple(5000,30);
        Motor_ForwardSimple(5000,50);
        Motor_LeftSimple(5000,30);
    }
}



void free_interrupt(void){
    Port2_Output(WHITE);
    while(1){
        Motor_ForwardSimple(5000,100);      // Moving in straight line.
    }
}



void mode_selection(void){
    while(!SW2IN && !SW1IN){            // Red LED blinking waiting for switches to be pressed
        SysTick_Wait10ms(10);
        REDLED = !REDLED;
    }
    REDLED = 0;
    if SW1IN{
        Port2_Output(SKYBLUE);          // Sky Blue color indicates auto mode is selected
        SysTick_Wait10ms(1000);
        while(!SW2IN && !SW1IN){        // Red LED blinking waiting for switches to be pressed for the second time
            SysTick_Wait10ms(10);
            REDLED = !REDLED;
        }
        if SW1IN{
            mode=1;                     // Auto mode using interrupt
        }
        if SW2IN{
            mode=2;                     // Auto mode using polling
        }
        REDLED = 0;
        Port2_Output(0);
    }
    if SW2IN{
        Port2_Output(PINK);             // Pink color indicates free motion mode is selected
        SysTick_Wait10ms(1000);
        while(!SW2IN && !SW1IN){        // Red LED blinking waiting for switches to be pressed for the second time.
            SysTick_Wait10ms(10);
            REDLED = !REDLED;
        }
        if SW1IN{
            mode=3;                     // Free motion mode using interrupt
        }
        if SW2IN{
            mode=4;                     // Free motion mode using polling
        }
        REDLED = 0;
        Port2_Output(0);
    }
}



void entering_mode(void){
    if (mode==1){
        auto_interrupt();
    }
    else if (mode==2){
        DisableInterrupts();
        auto_polling();
    }
    else if (mode==3){
        free_interrupt();
    }
    else if (mode==4){
        DisableInterrupts();
        free_polling();
    }
}



//void restart(void){
//    Clock_Init48MHz();        // Initialize clock with 48MHz frequency
//    Switch_Init();            // Initialize switches
//    SysTick_Init();           // Initialize SysTick timer
//    Port1_Init();             // Initialize P1.1 and P1.4 built-in buttons
//    Port2_Init();             // Initialize P2.2-P2.0 built-in LEDs
//    BumpEdgeTrigger_Init();   // Initialize bump switches using edge interrupt
//    mode_selection();         // Selecting mode (mode 1 is auto+interrupt, mode 2 is auto+polling, mode 3 is free+interrupt, mode 4 is free+polling)
//    Motor_InitSimple();       // Initialize DC Motor
//    Motor_StopSimple(10);     // Stop the motor on initial state
//    EnableInterrupts();       // Turn on interrupt
//    entering_mode();          // Entering the selected mode
//}



int main(void){
  Clock_Init48MHz();        // Initialize clock with 48MHz frequency
  Switch_Init();            // Initialize switches
  SysTick_Init();           // Initialize SysTick timer
  Port1_Init();             // Initialize P1.1 and P1.4 built-in buttons
  Port2_Init();             // Initialize P2.2-P2.0 built-in LEDs
  BumpEdgeTrigger_Init();   // Initialize bump switches using edge interrupt
  mode_selection();         // Selecting mode (mode 1 = auto+interrupt, mode 2 = auto+polling, mode 3 = free+interrupt, mode 4 = free+polling)
  Motor_InitSimple();       // Initialize DC Motor
  Motor_StopSimple(10);     // Stop the motor on initial state
  EnableInterrupts();       // Turn on interrupt
  entering_mode();          // Entering the selected mode
}
