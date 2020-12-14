/*
 * protection.c
 *
 *  Created on: 31 Mar. 2020
 *      Author: Worker
 */

#include "F28x_Project.h"     // Device Headerfile and Include File

#include "settings.h"
#include "protection.h"


#pragma CODE_SECTION(protectionDRVA_isr, ".TI.ramfunc")
interrupt void protectionDRVA_isr(void);

#pragma CODE_SECTION(protectionDRVB_isr, ".TI.ramfunc")
interrupt void protectionDRVB_isr(void);


Uint16 cntX1 = 0, cntX2 = 0;

void EnableProtection(void) {

    //
    // Enable XINT1 and XINT2
    //
    XintRegs.XINT1CR.all = XintRegs.XINT1CR.all | 0x1;            // Enable XINT1
    XintRegs.XINT2CR.all = XintRegs.XINT1CR.all | 0x1;            // Enable XINT2

}

void InitProtection(void) {

    //
    // GPIO0 and GPIO1 are inputs
    //
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;      // input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0;    // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPEGMUX1.bit.GPIO139 = 0;   // GPIO
    GpioCtrlRegs.GPEDIR.bit.GPIO139 = 0;     // input
    GpioCtrlRegs.GPEQSEL1.bit.GPIO139 = 0;   // XINT1 Synch to SYSCLKOUT only
    EDIS;

    //
    // GPIO0 is XINT1, GPIO1 is XINT2
    //
    GPIO_SetupXINT1Gpio(nFault_A);
    GPIO_SetupXINT2Gpio(nFault_B);

    //
    // Configure XINT1
    //
    XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 0;          // Falling edge interrupt

}


interrupt void protectionDRVA_isr(void) {

    cntX1++;



    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

interrupt void protectionDRVB_isr(void) {

    cntX2++;



    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}



