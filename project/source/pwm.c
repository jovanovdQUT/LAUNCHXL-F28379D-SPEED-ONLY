/*
 * pwm.c
 *
 *  Created on: 15 Feb. 2020
 *      Author: Dejan Jovanovic
 *
 *
 */

#include "F28x_Project.h"     // Device Headerfile and Include File

// The user-defined PWM functions
#include "pwm.h"
#include "settings.h"


// Rectifier
void SettingsEPWM1(void);
void SettingsEPWM2(void);
void SettingsEPWM3(void);

// DC/DC converter
void SettingsEPWM4(void);
void SettingsEPWM5(void);
void SettingsEPWM6(void);

void ConfigureEPWM(void) {

#if (CONFIG_AS_FULL_BRIDGE == 1)

    SettingsEPWM1();
    SettingsEPWM2();

    SettingsEPWM4();
    SettingsEPWM5();

#else

    SettingsEPWM1();
    SettingsEPWM2();
    SettingsEPWM3();

    // Debugging ports
    SettingsEPWM4();
    SettingsEPWM5();
    SettingsEPWM6();

#endif

}


void SettingsEPWM1(void) {

    EPwm1Regs.TBPRD = (Uint16) PWM_PERIOD;  /*20kHz*/

    EPwm1Regs.CMPA.bit.CMPA =  0; // EPwm1Regs.TBPRD / 2;

    EPwm1Regs.TBPHS.all = 0; // Set Phase register to zero

    EPwm1Regs.TBCTR = 0; // clear TB counter

    EPwm1Regs.TBCTL.bit.CTRMODE =   TB_COUNT_UPDOWN;    // Symmetric //TB_COUNT_UP; //
    EPwm1Regs.TBCTL.bit.PHSEN =     TB_DISABLE;         // Phase loading disabled - Master module
    EPwm1Regs.TBCTL.bit.PRDLD =     TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL =  TB_CTR_ZERO;


    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // TB_DIV1 =>
    EPwm1Regs.TBCTL.bit.CLKDIV =    TB_DIV1; // TB_DIV1 => TBCLK = SYSCLKOUT / 2

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // load on CTR = Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD; // load on CTR = Zero

    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE; // Reset

    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // ePWM1A = source for RED & FED

    EPwm1Regs.DBRED.all = DEAD_BAND;                // 4 microseconds delay
    EPwm1Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge

    EPwm1Regs.ETSEL.all = 0;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD; // interrupt on PRD
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;

    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;

}

void SettingsEPWM2(void) {

    EPwm2Regs.TBPRD = (Uint16) PWM_PERIOD;         // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm2Regs.CMPA.bit.CMPA = 0; // EPwm2Regs.TBPRD / 2;   // Set compare A value

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down //TB_COUNT_UP; //
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.SYNCOSEL =  TB_SYNC_IN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM2A on event
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM2A on event


    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // Reset
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;  //DBB_ALL; //       // ePWM2A = source for RED & FED
    EPwm2Regs.DBRED.all = DEAD_BAND;                // 2 microseconds delay
    EPwm2Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge


    EPwm2Regs.ETSEL.all = 0;

//    EPwm2Regs.ETSEL.bit.INTSEL =  ET_CTR_PRD; // interrupt on PRD
//    EPwm2Regs.ETSEL.bit.SOCAEN = DC_SOC_ENABLE;
//    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;               // interrupt enable for ePWM2
//
//    EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;

}


void SettingsEPWM3(void) {

    EPwm3Regs.TBPRD = (Uint16) PWM_PERIOD;         // Set timer period 801 TBCLKs
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm3Regs.CMPA.bit.CMPA = 0; // EPwm3Regs.TBPRD / 2;   // Set compare A value

    //
    // Setup counter mode
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down //TB_COUNT_UP; //
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.SYNCOSEL =  TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

    //
    // Set actions
    //
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM2A on event
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM2A on event


    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // Reset
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;  //DBB_ALL; //       // EPwm3A = source for RED & FED
    EPwm3Regs.DBRED.all = DEAD_BAND;                // 2 microseconds delay
    EPwm3Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge


    EPwm3Regs.ETSEL.all = 0;

//    EPwm3Regs.ETSEL.bit.INTSEL =  ET_CTR_PRD; // interrupt on PRD
//    EPwm3Regs.ETSEL.bit.SOCAEN = DC_SOC_ENABLE;
//    EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;               // interrupt enable for EPwm3
//
//    EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;

}



void SettingsEPWM4(void) {

    EPwm4Regs.TBPRD = (Uint16) PWM_PERIOD;         // Set timer period 801 TBCLKs
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm4Regs.CMPA.bit.CMPA = EPwm4Regs.TBPRD / 2;   // Set compare A value

    //
    // Setup counter mode
    //
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm4Regs.TBCTL.bit.SYNCOSEL =  TB_SYNC_IN;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

    //
    // Set actions
    //
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM4A on event
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM4A on event


    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // Reset
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;  //DBB_ALL; //       // ePWM4A = source for RED & FED
    EPwm4Regs.DBRED.all = DEAD_BAND;                // 2 microseconds delay
    EPwm4Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge


    EPwm4Regs.ETSEL.all = 0;

//    EPwm4Regs.ETSEL.bit.INTSEL =  ET_CTR_PRD; // interrupt on PRD
//    EPwm4Regs.ETSEL.bit.SOCAEN = DC_SOC_ENABLE;
//    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
//
//    EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;

}


void SettingsEPWM5(void) {

    EPwm5Regs.TBPRD = (Uint16) PWM_PERIOD;         // Set timer period 801 TBCLKs
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD / 2;   // Set compare A value

    //
    // Setup counter mode
    //
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm5Regs.TBCTL.bit.SYNCOSEL =  TB_SYNC_IN;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

    //
    // Set actions
    //
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM5A on event
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM5A on event


    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // Reset
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;  //DBB_ALL; //       // ePWM5A = source for RED & FED
    EPwm5Regs.DBRED.all = DEAD_BAND;                // 2 microseconds delay
    EPwm5Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge


    EPwm5Regs.ETSEL.all = 0;

//    EPwm5Regs.ETSEL.bit.INTSEL =  ET_CTR_PRD; // interrupt on PRD
//    EPwm5Regs.ETSEL.bit.SOCAEN = DC_SOC_ENABLE;
//    EPwm5Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
//
//    EPwm5Regs.ETPS.bit.SOCAPRD = ET_1ST;

}



void SettingsEPWM6(void) {

    EPwm6Regs.TBPRD = (Uint16) PWM_PERIOD;         // Set timer period 801 TBCLKs
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm6Regs.CMPA.bit.CMPA = EPwm6Regs.TBPRD / 2;   // Set compare A value

    //
    // Setup counter mode
    //
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm6Regs.TBCTL.bit.SYNCOSEL =  TB_SYNC_IN;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

    //
    // Set actions
    //
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM5A on event
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;          // Set PWM5A on event


    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;  // Reset
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active High complementary
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;  //DBB_ALL; //       // ePWM5A = source for RED & FED
    EPwm6Regs.DBRED.all = DEAD_BAND;                // 2 microseconds delay
    EPwm6Regs.DBFED.all = DEAD_BAND;                // for rising and falling edge


    EPwm6Regs.ETSEL.all = 0;

//    EPwm6Regs.ETSEL.bit.INTSEL =  ET_CTR_PRD; // interrupt on PRD
//    EPwm6Regs.ETSEL.bit.SOCAEN = DC_SOC_ENABLE;
//    EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
//
//    EPwm6Regs.ETPS.bit.SOCAPRD = ET_1ST;

}
