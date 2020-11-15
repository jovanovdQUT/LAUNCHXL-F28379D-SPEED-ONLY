/*
 * main.c
 *
 *  Created on: 12 Feb. 2020
 *      Author: Dejan Jovanovic
 */




//
// Included Files
//
#include "F28x_Project.h"

#include "settings.h"
#include "pwm.h"
#include "adc.h"
#include "control.h"

void main(void) {

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    //
    InitSysCtrl();

    // Disable CPU interrupts
    DINT;

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it’s default state.
    //
    InitGpio();

    GPIO_SetupPinMux(BLINKY_LED_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(TEST_PIN, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TEST_PIN, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(WAKE_A, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(WAKE_A, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(WAKE_B, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(WAKE_B, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();


    //
    // Step 4. Initialized PWM
    InitEPwmGpio();
    ConfigureEPWM();

    //
    // Step 5. Initialized ADC
    Adc_Config();

    //
    // Step 6. Initialize control
    InitControl();



//    //
//    // Step 7. Initialize communication
//    InitComms();
//
//    InitComms();
//
//    //
//    // Step 8. Initialize protection based nFAULT signals from DRV8305
//    InitProtection();
//    EnableProtection();

    // Enable global Interrupts and higher priority real-time debug events:
    EnableInterrupts();


    for(;;) {

        //
        // Turn on LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO, RESET);

        //
        // Delay for a bit.
        //
        DELAY_US(1000*1000);
        //
        // Turn off LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO, SET);


        //
        // Delay for a bit.
        //
        DELAY_US(1000*1000);

    }

}
