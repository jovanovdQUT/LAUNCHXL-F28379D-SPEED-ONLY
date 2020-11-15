/*
 * DRV8305_SPI.c
 *
 *  Created on: 31 Mar. 2020
 *      Author: Worker
 */




//============================================================================
//============================================================================
//
// FILE:    DRV8305_SPI.c
//
// TITLE:   DRV8305 SPI comm functions
//
// Version: 1.0
//
// Date:    24 May 2011
//
//============================================================================
//============================================================================
#include "F28x_Project.h"     // Device Headerfile and Include File
#include "DRV8305_SPI.h"

#include "settings.h"

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 5 MHz
#define SPI_CLK         (5E6)
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR        ((((200E6) / (4)) / (SPI_CLK)) - (1))
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

// SPI variables
union DRV8305_STATUS_REG_1 DRV8305_stat_reg1_spia;
union DRV8305_STATUS_REG_2 DRV8305_stat_reg2_spia;

union DRV8305_CONTROL_REG_1 DRV8305_cntrl_reg1_spia;
union DRV8305_CONTROL_REG_2 DRV8305_cntrl_reg2_spia;

union DRV8305_STATUS_REG_1 DRV8305_stat_reg1_spib;
union DRV8305_STATUS_REG_2 DRV8305_stat_reg2_spib;

union DRV8305_CONTROL_REG_1 DRV8305_cntrl_reg1_spib;
union DRV8305_CONTROL_REG_2 DRV8305_cntrl_reg2_spib;

Uint16 read_drv_status = 0;

/*****************************************************************************/
// Initialize the SPI peripheral
/*****************************************************************************/
void DRV8305_SPI_Init(volatile struct SPI_REGS *s)
{
    s->SPICCR.bit.SPISWRESET = 0;       // Put SPI in reset state
    s->SPICCR.bit.SPICHAR = 0xF;        // 16-bit character
    s->SPICCR.bit.SPILBK = 0;           // Loopback off
    s->SPICCR.bit.CLKPOLARITY = 0;      // Rising edge without delay

    s->SPICTL.bit.SPIINTENA = 0;        // disable SPI interrupt
    s->SPICTL.bit.TALK = 1;             // enable transmission
    s->SPICTL.bit.MASTER_SLAVE = 1;     // master
    s->SPICTL.bit.CLK_PHASE = 0;        // Rising edge without delay
    s->SPICTL.bit.OVERRUNINTENA = 0;    // disable reciever overrun interrupt

//    s->SPIBRR = 0;                      // SPICLK = LSPCLK / 4 (max SPICLK)
    s->SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    s->SPICCR.bit.SPISWRESET=1;         // Enable SPI
}

/*****************************************************************************/
// Read from a DRV8305 Register
/*****************************************************************************/
Uint16 DRV8305_SPI_Read(volatile struct SPI_REGS *s, Uint16 address)
{
    union DRV8305_SPI_WRITE_WORD_REG w;
    volatile Uint16 dummy;

    w.bit.R_W = 1;                          //we are initiating a read
    w.bit.ADDRESS = address;                //load the address
    w.bit.DATA = 0;                         //dummy data;

    s->SPITXBUF = w.all;                    //send out the data

    while(s->SPISTS.bit.INT_FLAG == 0);     //wait for the packet to complete

    dummy = s->SPIRXBUF;                    //dummy read to clear the INT_FLAG bit

    w.bit.R_W = 1;                          //we are initiating a read
    w.bit.ADDRESS = address;                //load the address
    w.bit.DATA = 0;                         //dummy data;

    s->SPITXBUF = w.all;                    //send out the data

    while(s->SPISTS.bit.INT_FLAG == 0);     //wait for the packet to complete

    dummy = s->SPIRXBUF;                    //dummy read to clear the INT_FLAG bit

    return(dummy);

}

/*****************************************************************************/
// Write to a DRV8305 Register
// SPI writes always clock out the data in Status Register 1.
// Since it's available we'll return the status from this function
/*****************************************************************************/
Uint16 DRV8305_SPI_Write(volatile struct SPI_REGS *s, Uint16 address, Uint16 data)
{
    union DRV8305_SPI_WRITE_WORD_REG w;
    volatile Uint16 stat_reg1;

    w.bit.R_W = 0;                          //we are initiating a write
    w.bit.ADDRESS = address;                //load the address
    w.bit.DATA = data;                      //data to be written;

    s->SPITXBUF = w.all;                    //send out the data

    while(s->SPISTS.bit.INT_FLAG == 0);     //wait for the packet to complete

    stat_reg1 = s->SPIRXBUF;                //read returned value of Status Register 1 and clear the INT_FLAG bit

    return(stat_reg1);

}
