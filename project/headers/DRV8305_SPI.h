/*
 * DRV8305_SPI.h
 *
 *  Created on: 31 Mar. 2020
 *      Author: Dejan Jovanovic
 */

#ifndef PROJECT_HEADERS_DRV8305_SPI_H_
#define PROJECT_HEADERS_DRV8305_SPI_H_

//============================================================================
//============================================================================
//
// FILE:    DRV8305_SPI.h
//
// TITLE:   Header file for DRV8305 SPI comm functions
//
// Version: 1.0
//
// Date:    24 May 2011
//
//============================================================================
//============================================================================

// DRV8305 SPI Input Data bit definitions:
struct  DRV8305_SPI_WRITE_WORD_BITS {       // bit      description
   Uint16 DATA:11;                          // 10:0     FIFO reset
   Uint16 ADDRESS:4;                        // 14:11    Enhancement enable
   Uint16 R_W:1;                            // 15       R/W
};

union DRV8305_SPI_WRITE_WORD_REG {
   Uint16                               all;
   struct DRV8305_SPI_WRITE_WORD_BITS   bit;
};

// DRV8305 SPI Status Reister 1 bit definitions:
struct  DRV8305_STATUS_REG_1_BITS {     // bit      description
   Uint16 OTW:1;                        // 0        Over-temperature warning
   Uint16 TEMP_FLAG3:1;                 // 1        Temperature flag setting for approximately 135°C
   Uint16 TEMP_FLAG2:1;                 // 2        Temperature flag setting for approximately 125°C
   Uint16 TEMP_FLAG1:1;                 // 3        Temperature flag setting for approximately 105°C
   Uint16 VCHP_UVFL:1;                  // 4        Charge pump under voltage flag warning
   Uint16 VDS_STATUS:1;                 // 5        Real time OR of all VDS over current monitors
   Uint16 PVDD_OVFL:1;                  // 6        PVDD over voltage flag warning
   Uint16 PVDD_UVFL:1;                  // 7        PVDD under voltage flag warning
   Uint16 TEMP_FLAG4:1;                 // 8        Temperature flag setting for approximately 175°C
   Uint16 RSVD:1;                       // 9
   Uint16 FAULT:1;                      // 10       FAULTn pin is asserted
   Uint16 Reserved:5;                   // 15:11
};


union DRV8305_STATUS_REG_1 {
   Uint16                           all;
   struct DRV8305_STATUS_REG_1_BITS bit;
};

// DRV8305 SPI Status Reister 2 bit definitions:
struct  DRV8305_STATUS_REG_2_BITS {     // bit      description
   Uint16 SNS_A_OCP:1;                  // 0        Sense A over current fault
   Uint16 SNS_B_OCP:1;                  // 1        Sense B over current fault
   Uint16 SNS_C_OCP:1;                  // 2        Sense C over current fault
   Uint16 RSVD:2;                       // 4:3
   Uint16 VDS_LC:1;                     // 5        VDS over current fault for low-side MOSFET C
   Uint16 VDS_HC:1;                     // 6        VDS over current fault for high-side MOSFET C
   Uint16 VDS_LB:1;                     // 7        VDS over current fault for low-side MOSFET B
   Uint16 VDS_HB:1;                     // 8        VDS over current fault for high-side MOSFET B
   Uint16 VDS_LA:1;                     // 9        VDS over current fault for low-side MOSFET A
   Uint16 VDS_HA:1;                     // 10       VDS over current fault for high-side MOSFET A
   Uint16 Reserved:5;                   // 15:11
};

union DRV8305_STATUS_REG_2 {
   Uint16                           all;
   struct DRV8305_STATUS_REG_2_BITS bit;
};


// DRV8305 SPI Control Reister 1 bit definitions:
struct  DRV8305_CONTROL_REG_1_BITS {    // bit      description
   Uint16 GATE_CURRENT:2;               // 1:0      Gate driver peak current, 1.7A (00b), 0.7A (01b), 0.25A (10b), Reserved (11b)
   Uint16 GATE_RESET:1;                 // 2        Reset all latched faults (1), Normal Mode (0)
   Uint16 PWM_MODE:1;                   // 3        Three (1) or six (0) pwm inputs
   Uint16 OC_MODE:2;                    // 5:4      over-current mode, current limit (00b), latched shut down (01b), Report only (10b), OC disabled (11b)
   Uint16 OC_ADJ_SET:5;                 // 10:6     Set Vds trip point for OC see the table in the datasheet
   Uint16 Reserved:5;                   // 15:11
};

union DRV8305_CONTROL_REG_1 {
   Uint16                               all;
   struct DRV8305_CONTROL_REG_1_BITS    bit;
};

// DRV8305 SPI Control Reister 2 bit definitions:
struct  DRV8305_CONTROL_REG_2_BITS {    // bit      description
   Uint16 OCTW_SET:2;                   // 1:0      Report OT and OC (00b), Report OT (01b), Report OC (10b), Reserved (11b)
   Uint16 GAIN:2;                       // 3:2      Gain of shunt amplifier, 10 (00b), 20 (01b), 40 (10b), 80 (11b)
   Uint16 DC_CAL_CH1:1;                 // 4        Shunt amplifier inputs shorted and disconnected from load (1) or shunt amplifier inputs connected to load (0)
   Uint16 DC_CAL_CH2:1;                 // 5        Shunt amplifier inputs shorted and disconnected from load (1) or shunt amplifier inputs connected to load (0)
   Uint16 OC_TOFF:1;                    // 6        Normal CBC operation (0), off time control during OC (1)
   Uint16 Reserved:9;                   // 15:7
};

union DRV8305_CONTROL_REG_2 {
   Uint16                               all;
   struct DRV8305_CONTROL_REG_2_BITS    bit;
};



/***************************************************************************************************/
//defines
/***************************************************************************************************/
//DRV8305 Register Addresses
#define STAT_REG_1_ADDR     0x1
#define STAT_REG_2_ADDR     0x2
#define STAT_REG_3_ADDR     0x3
#define STAT_REG_4_ADDR     0x4
#define STAT_REG_5_ADDR     0x5
#define STAT_REG_6_ADDR     0x6
#define STAT_REG_7_ADDR     0x7
#define STAT_REG_8_ADDR     0x8
#define STAT_REG_9_ADDR     0x9
#define STAT_REG_A_ADDR     0xA
#define STAT_REG_B_ADDR     0xB
#define STAT_REG_C_ADDR     0xC




#define CNTRL_REG_1_ADDR    0x02
#define CNTRL_REG_2_ADDR    0x03

/***************************************************************************************************/
//function prototypes
/***************************************************************************************************/
void DRV8305_SPI_Init(volatile struct SPI_REGS *s);

Uint16 DRV8305_SPI_Read(volatile struct SPI_REGS *s, Uint16 address);
Uint16 DRV8305_SPI_Write(volatile struct SPI_REGS *s, Uint16 address, Uint16 data);





#endif /* PROJECT_HEADERS_DRV8305_SPI_H_ */


//// DRV8305 SPI Status Reister 1 bit definitions:
//struct  DRV8305_STATUS_REG_1_BITS {     // bit      description
//   Uint16 FETLC_OC:1;                   // 0        Phase C, low-side FET OC
//   Uint16 FETHC_OC:1;                   // 1        Phase C, high-side FET OC
//   Uint16 FETLB_OC:1;                   // 2        Phase B, low-side FET OC
//   Uint16 FETHB_OC:1;                   // 3        Phase B, high-side FET OC
//   Uint16 FETLA_OC:1;                   // 4        Phase A, low-side FET OC
//   Uint16 FETHA_OC:1;                   // 5        Phase A, high-side FET OC
//   Uint16 OTW:1;                        // 6        Over-temperature warning
//   Uint16 OTSD:1;                       // 7        Over-temperature shut down
//   Uint16 PVDD_UV:1;                    // 8        PVDD < 6V
//   Uint16 GVDD_UV:1;                    // 9        GVDD < 8V
//   Uint16 FAULT:1;                      // 10       FAULTn pin is asserted
//   Uint16 Reserved:5;                   // 15:11
//};
