/*
 * comms.c
 *
 *  Created on: 7 Mar. 2020
 *      Author: Dejan Jovanovic
 */
#include "F28x_Project.h"     // Device Headerfile and Include File

#include "comms.h"


Uint16 spi_tx_data[4] = {1, 2, 3, 4};

//void SpiInit(void);

void InitComms(void) {

    spi_tx_data[0] = 0;

}


//
//void SpiInit() {
//
//    SpiaRegs.SPICCR.bit.SPISWRESET = 0;                 // Hold SPI in reset
//    SpiaRegs.SPICCR.bit.SPICHAR = 15;                   // 16 bit char
//    SpiaRegs.SPICCR.bit.CLKPOLARITY = RISING_EDGE_OUT;  // Output on rising edge
//    SpiaRegs.SPICCR.bit.SPILBK = NO_LOOPBACK;           // No Loopback
//    SpiaRegs.SPICCR.bit.SPISWRESET = ENABLE;                    // Release SPI from reset
//
//
//    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0;
////    SpiaRegs.SPIBRR = 0;
//    // SPI Bit Rate =  LSPCLK / ( SPIBRR + 1)
//    //              =  LSPCLK / 4
//    //              =  37.5 MHz (26.667ns)
//
//    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;       // Master mode
//    SpiaRegs.SPICTL.bit.CLK_PHASE = DISABLE;    // No Delay
//    SpiaRegs.SPICTL.bit.OVERRUNINTENA = DISABLE;// Disable
//    SpiaRegs.SPICTL.bit.TALK = 1;               // Enable TX
//    SpiaRegs.SPICTL.bit.SPIINTENA = DISABLE;    // Disable non-FIFO Interrupt Request //ENABLE; // Enable Interrupt Request
//
//    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0; //ENABLE;    // Tx FIFO Enable
//    SpiaRegs.SPIFFTX.bit.TXFFIL = 0x04;     // Transmit 4 words
//    SpiaRegs.SPIFFTX.bit.SPIFFENA = ENABLE; // SPI FIFO enhancements enable
//    SpiaRegs.SPIFFTX.bit.TXFIFO = RESET_TXFIFO;
//
//    SpiaRegs.SPIFFCT.bit.TXDLY = 10;
//
//    // Interrupts that are used in this example are re-mapped to
//    // ISR functions found within this file.
////  EALLOW; // This is needed to write to EALLOW protected registers
////  PieVectTable.SPITXINTA = &spi_tx_fifo_isr;
////  EDIS;   // This is needed to disable write to EALLOW protected registers
////
////  PieCtrlRegs.PIEIER6.bit.INTx2 = ENABLE;         // Enable PIE Group 6, INT 2
////  IER |= M_INT6;                                  // Enable INT6 for SPI-A
//
//}
