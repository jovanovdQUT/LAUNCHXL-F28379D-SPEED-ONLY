//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.08.00.00 $
// $Release Date: Mon Dec 23 17:32:30 IST 2019 $
// $Copyright:
// Copyright (C) 2013-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"

#include "settings.h"
//#include "DRV8305_SPI.h"
////
//// Calculate BRR: 7-bit baud rate register value
//// SPI CLK freq = 5 MHz
//#define SPI_CLK         (5E6)
//// LSPCLK freq  = CPU freq / 4  (by default)
//// BRR          = (LSPCLK freq / SPI CLK freq) - 1
////
//#if CPU_FRQ_200MHZ
//#define SPI_BRR        ((((200E6) / (4)) / (SPI_CLK)) - (1))
//#endif
//
//#if CPU_FRQ_150MHZ
//#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
//#endif
//
//#if CPU_FRQ_120MHZ
//#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
//#endif

#pragma CODE_SECTION(spia_rx_isr, ".TI.ramfunc")
interrupt void spia_rx_isr(void);

Uint16 rdata[2];     // Receive data buffer

//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void) {

    // Initialize SPI-A
//    DRV8305_SPI_Init(&SpiaRegs);


    // Initialize SPI-B
//    DRV8305_SPI_Init(&SpibRegs);

//    // Step 1: Clear the SPI Software Reset bit (SPISWRESET) to 0 to force the SPI to the reset state.
//    // Set reset low before configuration changes
//    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
//
//    // Step 2: Configure the SPI as desired
//    // Clock polarity (0 == rising, 1 == falling)
//    // 16-bit character
//    // Disable loop-back
//    // Disable high-speed mode
//    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
//    SpiaRegs.SPICCR.bit.SPICHAR = 15;
//    SpiaRegs.SPICCR.bit.SPILBK = 0;
//    SpiaRegs.SPICCR.bit.HS_MODE = 0;
//
//    // Enable master (0 == slave, 1 == master)
//    // Enable transmission (Talk == 1)
//    // Clock phase (0 == normal, 1 == delayed)
//    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
//    SpiaRegs.SPICTL.bit.TALK = 1;
//    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
//
//
//    // Set the baud rate
//    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;
//
//    // Set FREE bit
//    // Halting on a breakpoint will not halt the SPI
////    SpiaRegs.SPIPRI.bit.FREE = 1;
//
//    // Step 3: Enable interrupts
//    // SPI interrupts are enabled
//    // SPI FIFO enhancements enable
//    SpiaRegs.SPICTL.bit.SPIINTENA = DISABLE;    // Disable non-FIFO Interrupt Request
//    SpiaRegs.SPIFFTX.bit.SPIFFENA = ENABLE;
//    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = RESET_FIFO;
//    SpiaRegs.SPIFFTX.bit.TXFFIENA = DISABLE;
//    SpiaRegs.SPIFFTX.bit.TXFFIL = 0; // TX interrupt when 0 words is left in TXBUF
//
//    SpiaRegs.SPIFFRX.bit.RXFFIENA = ENABLE;
//    SpiaRegs.SPIFFRX.bit.RXFFIL = 0x1; // RX interrupt if 1 or more words are received
//    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = RESET_FIFO;
//    SpiaRegs.SPIFFRX.bit.RXFIFORESET = RESET_FIFO; // Reset FIFO
//
//    // Step 4: Set SPISWRESET to 1 to release the SPI from the reset state
//    // Release the SPI from reset
//    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.SPIA_RX_INT = &spia_rx_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    PieCtrlRegs.PIEIER6.bit.INTx1 = ENABLE;         // Enable PIE Group 6, INT 2
    IER |= M_INT6;                                  // Enable INT6 for SPI-A

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



}

interrupt void spia_rx_isr(void) {

    Uint16 i;

    GPIO_WritePin(TEST_PIN, 1);



    for(i = 0; i < 1; i++) {

        rdata[i] = SpiaRegs.SPIRXBUF;     // Read data
    }

    GPIO_WritePin(TEST_PIN, 0);

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag

    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//
void InitSpiGpio() {

   InitSpiaGpio();
   InitSpibGpio();

}

//
// InitSpiaGpio - Initialize SPIA GPIOs
//
void InitSpiaGpio(void)
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //

    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;      // Enable pull-up on GPIO58 (SPISIMOA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;      // Enable pull-up on GPIO16 (SPISIMOA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;       // Enable pull-up on GPIO5 (SPISIMOA)

    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;     // Enable pull-up on GPIO59 (SPISOMIA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;     // Enable pull-up on GPIO17 (SPISOMIA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;      // Enable pull-up on GPIO3 (SPISOMIA)

    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;     // Enable pull-up on GPIO18 (SPICLKA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     // Enable pull-up on GPIO18 (SPICLKA)

    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;     // Enable pull-up on GPIO19 (SPISTEA)
//  GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     // Enable pull-up on GPIO19 (SPISTEA)

    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;   // Asynch input GPIO58 (SPISIMOA)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO59 (SPISOMIA)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO60 (SPICLKA)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;   // Asynch input GPIO61 (SPISTEA)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA
//  GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
//  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;  // Configure GPIO5 as SPISIMOA

    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;    // Configure GPIO59 as SPISOMIA
//  GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
//  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;  // Configure GPIO3 as SPISOMIA



    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;    // Configure GPIO60 as SPICLKA
//  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA


    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;  // Configure GPIO61 as SPISTEA
//  GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

    EDIS;
}

//
// InitSpibGpio - Initialize SPIB GPIOs
//
void InitSpibGpio(void)
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //


    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable pull-up on GPIO63 (SPISIMOA)

    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;  // Enable pull-up on GPIO64 (SPISOMIA)

    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;  // Enable pull-up on GPIO65 (SPICLKA)

    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;  // Enable pull-up on GPIO66 (SPISTEA)

    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Asynch input GPIO63 (SPISIMOA)

    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Asynch input GPIO64 (SPISOMIA)

    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Asynch input GPIO65 (SPICLKA)

    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3; // Asynch input GPIO66 (SPISTEA)

    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3; // Configure GPIO16 as SPISIMOA

    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3; // Configure GPIO17 as SPISOMIA

    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3; // Configure GPIO18 as SPICLKA

    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3; // Configure GPIO19 as SPISTEA

    EDIS;
}


//
// End of file
//
