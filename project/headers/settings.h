/*
 * settings.h
 *
 *  Created on: 15 Feb. 2020
 *      Author: Dejan Jovanovic
 */

#ifndef PROJECT_HEADERS_SETTINGS_H_
#define PROJECT_HEADERS_SETTINGS_H_


#define PI          (3.1415926535897932385)
#define PI_INV      (0.3183)
#define TWO_PI      (6.28318531)
#define TWOPI_INV   (0.591549)

#define INVSQRT3    (0.5774)

#define FALSE       (0)
#define TRUE        (1)

#define RESET       (0)
#define SET         (1)

// BOOSTXL-DRV8305EVM control signals. For the booster pack signal list check Embedded Software Design documentation

#define TEST_PIN        (32)        // Used to debug the ADC ISR

#define BLINKY_LED_GPIO (31)        // GPIO31
#define LED_RED         (34)        // GPIO34

#define EN_GATE_A       (124)       // GPIO124
#define EN_GATE_B       (26)        // GPIO26

#define WAKE_A          (124)       // GPIO124
#define WAKE_B          (26)        // GPIO27

#define nFault_A        (19)        // GPIO19
#define nFault_B        (139)       // GPIO139

#define SCLK_A          (60)        // GPIO60
#define SCLK_B          (65)        // GPIO60

#define SPI_CS_A        (61)        // GPIO61
#define SPI_CS_B        (66)        // GPIO66

#define SDI_A           (58)        // GPIO58
#define SDI_B           (63)        // GPIO63

#define SDO_A           (59)        // GPIO59
#define SDO_B           (64)        // GPIO64


// PWM settings

#if CPU_FRQ_200MHZ

    #define CPU_CLOCK_FREQ      (200e6)

#endif

#define NUMBER_OF_PP        (5.0)
#define Fo_MAX              (100.00)

#define FLUX_Mo             (0.00921768)
#define FLUX_M              ((7.5)*(FLUX_Mo))    // (3P/2)*Flux_nom

#define SYNCHRO_BOUNDARY    (((Fo_MAX)-(0.25))*(TWO_PI))
#define SYNCHRO_RANGE        ((Fo_MAX)*(TWO_PI))

#define PWM_CLOCK_FREQ      ((CPU_CLOCK_FREQ)/(2.0))          // Feature of the chip, section 15.4.1 or page 1871 of technical manual SPRUHM8I–December 2013–Revised September 2019

#define ISR_FREQUENCY       (20.0e3)

#define DELTA_T             (((1.0)/(ISR_FREQUENCY)))       // Switching frequency Fsw = 20kHz (0.00005s) Fsw = 10kHz (0.0001s) Fsw = 5kHz (0.0002s)

#define PWM_PERIOD      (((PWM_CLOCK_FREQ)/((2.0)*(ISR_FREQUENCY)))) //(2500) //

#define DEAD_BAND       (15) // 150ns dead-time @PWM_CLOCK_FREQ 100MHz

#define GRID_FREQ       (50.0)
#define GRID_FREQ_MIN   (45.0)
#define GRID_FREQ_MAX   (55.0)

#define SYNCHRO_TIME_MAX_SEC    ((1.0) * (ISR_FREQUENCY) * (DELTA_T))

#define STATE_CHNAGE_5SEC   (100000)
#define VDC_CTRL            (10)

#define DELTA_VDC_T         ( ((float)(VDC_CTRL))*(DELTA_T))

// ADC settings
#define VDC_MAX             (35.0)
#define VOLT_SCALE          ((1.0)/(VDC_MAX))

#define IDC_MAX             (1.0)

#define TRIGSEL_PWM1_SOC    (0x05)
#define TRIGSEL_PWM2_SOC    (0x07)
#define TRIGSEL_PWM3_SOC    (0x09)

//#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  (0x0)   // S/H width in ADC module periods                        = 16 ADC clocks


#define RISING_EDGE_OUT     (0)
#define NO_LOOPBACK         (0)
#define ENABLE              (1)
#define DISABLE             (0)
#define RESET_FIFO          (1)

#endif /* PROJECT_HEADERS_SETTINGS_H_ */
