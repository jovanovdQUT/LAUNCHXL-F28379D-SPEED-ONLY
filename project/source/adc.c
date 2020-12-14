/*
 * adc.c
 *
 *  Created on: 22 Feb. 2020
 *      Author: Dejan Jovanovic
 *
 *
 */


#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "Solar_F.h"

#include "settings.h"
#include "adc.h"
#include "control.h"
#include "filter.h"
#include "PQ.h"


#pragma CODE_SECTION(adc_isr, ".TI.ramfunc")
#pragma INTERRUPT(adc_isr , HPI)
interrupt void adc_isr(void);

extern SPLL_1ph_SOGI_F spll1;
extern Control_State_t control_state_info;
extern Synchro_Info_t synchro_info;
extern CLARKE_F cv, ci;
extern PARK_F pv, pi;
extern iPARK_F ipv;
extern SMOPOS_F smopos;
extern SPEED_ESTIMATION_F speedest;
extern PID_GRANDO_F_CONTROLLER torquePID, idPID, iqPID;
extern Uint32 state_change_5sec, vdc_ctrl_cnt;
extern float wRefRPM;
extern PID pidD, pidQ, pidS;

extern motor_params_t motor_params;
extern fll_info_t fll, fll_r;
extern resonant_t emfa, emfb, emfa_r, emfb_r;
extern pd_info_t pd_info, pd_info_r;
extern emf_info_t emf;
extern flux_info_t flux_r, flux_s;
extern synchro_3f_info_t synchro_3f;
extern sample_info_t sample_info;

extern resonant_t smo_a, smo_b;

extern dahlin_info_t dahlin_info;

// Nominal parameters
float IQ_REF = 0.5;

// Old values

float KpoD = 10.0,   KioD = 4.0;
float KpoQ = 10.0,   KioQ = 4.0;
float KpoS = 0.005, KioS = 5.0;

//float KpoD = 5.0, KioD = 0.1;
//float KpoQ = 5.0, KioQ = 0.1;
//float KpoS = 0.2, KioS = 0.005;

// New values
//float KpoD = 0.0324, KioD = 0.0017;
//float KpoQ = 0.0324, KioQ = 0.0017;
//float KpoS = 57.28, KioS = 9.92;

//float KpoD = 0.0324, KioD = 0.0017;
//float KpoQ = 0.0324, KioQ = 0.0017;
//float KpoS = 57.28, KioS = 9.92;

float cnt_q_ref = 0.0;
float torque_ref = 0.0;

float a0 = 0.0, a2 = 0.0, a3 = 0.0, b2 = 0.0, b3 = 0.0, c2 = 0.0, c3 = 0.0;

float c4 = 0.0, b4 = 0.0, a4 = 0.0, a1 = 0.0;

float temp = 0.0,  temp1 = 0.0, modul = 0.2, modulo = 0.2, tempTheta = 0.0, tempAngle = 0.0, tempAngle_1 = 0.0, signSpeed = 0.0, tempAngleE = 0.0, ramp_k = 0.0, ramp_k_1 = 0.0;
float xa_pos = 0.0, xb_pos = 0.0, xa_neg = 0.0, xb_neg = 0.0;
float x_pos = 0.0, x_neg = 0.0;


float frotor = 10.0;
float wrotor = 0;
float Nrotor = (ISR_FREQUENCY) / (GRID_FREQ);
float torque_est = 0, torque_est_1 = 0, ft = 1e-3;

Svgen_t svgn = {0};

ADC_Calibra_t calibra = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1000};

ADC_Params_t adc_calibra_params = { 2.917, 1.54,
                                    10.0, 0.0,
                                    2.917, 1.54,
                                    10.0, 0.0,
                                    2.917, 1.54,
                                    10.0, 0.0};

ADC_Read_t adc_read = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
ADC_Read_Norm_t adc_norm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



int k = 0, j = 0, state = 0;

void Adc_Config(void) {

    Uint16 acqps;

    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /1
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /1
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /1
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /1

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Interrupt pulse generation occurs at the end of the conversion
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14; //75ns //
    }
    else { //resolution is 16-bit
        acqps = 63; //320ns
    }


    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14; //75ns
    }
    else { //resolution is 16-bit
        acqps = 63; //320ns
    }

    if(ADC_RESOLUTION_12BIT == AdccRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14; //75ns
    }
    else { //resolution is 16-bit
        acqps = 63; //320ns
    }

    if(ADC_RESOLUTION_12BIT == AdcdRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14; //75ns
    }
    else { //resolution is 16-bit
        acqps = 63; //320ns
    }

    EALLOW;

    // CASE A
    PieVectTable.ADCD1_INT = &adc_isr;              // function for ADC D1 interrupt

//    PieVectTable.ADCD2_INT = &adc_isr;              // function for ADCD interrupt 2
//    PieVectTable.ADCD3_INT = &adc_isr;              // function for ADCD interrupt 3

    // CASE D
//    PieVectTable.ADCD4_INT = &adc_isr;              // function for ADCD interrupt 4

    EDIS;


    EALLOW;

    // Read gate driver temperature

    // CASE A
//    AdcaRegs.ADCSOC15CTL.bit.CHSEL = 0;          //  ADC A SOC15 will convert pin A0
//    AdcaRegs.ADCSOC15CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
//    AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C


    // CASE D
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;          //  ADC A SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;          //  ADC A SOC1 will convert pin A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C



    /********************************************************************************************/
    /* Read voltage reference from RTDS                                                         */
    /********************************************************************************************/
    // Read voltage reference phase n
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;          //  ADC A SOC2 will convert pin A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read voltage reference phase c
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;          //  ADC A SOC3 will convert pin A3
    AdcaRegs.ADCSOC3CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read voltage reference phase b
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;          //  ADC A SOC4 will convert pin A4
    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read voltage reference phase a
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;          //  ADC A SOC5 will convert pin A5
    AdcaRegs.ADCSOC5CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC14CTL.bit.CHSEL = 14;          //  ADC A SOC14 will convert pin A14
    AdcaRegs.ADCSOC14CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C

    /********************************************************************************************/
    /* Read filtered output current                                                             */
    /********************************************************************************************/
    // Read filtered output current phase n
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;          //  ADC B SOC2 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read filtered output current phase c
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;          //  ADC B SOC3 will convert pin B3
    AdcbRegs.ADCSOC3CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read filtered output current phase b
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 4;          //  ADC B SOC4 will convert pin B4
    AdcbRegs.ADCSOC4CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read filtered output current phase a
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;          //  ADC B SOC5 will convert pin B5
    AdcbRegs.ADCSOC5CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C

    /********************************************************************************************/
    /* Read filter capacitor voltage                                                            */
    /********************************************************************************************/
    //  Read capacitor voltage phase n
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 2;          //  ADC C SOC2 will convert pin C2
    AdccRegs.ADCSOC2CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    //  Read capacitor voltage phase c
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 3;          //  ADC C SOC3 will convert pin C3
    AdccRegs.ADCSOC3CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    //  Read capacitor voltage phase b
    AdccRegs.ADCSOC4CTL.bit.CHSEL = 4;          //  ADC C SOC4 will convert pin C4
    AdccRegs.ADCSOC4CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    //  Read capacitor voltage phase a
    AdccRegs.ADCSOC5CTL.bit.CHSEL = 5;          //  ADC C SOC5 will convert pin C5
    AdccRegs.ADCSOC5CTL.bit.ACQPS  = acqps;     //  sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C

    /********************************************************************************************/
    /* Read inverter output current                                                             */
    /********************************************************************************************/
    // Read inverter output current phase n
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;                  //  ADC D SOC2 will convert pin D2
    AdcdRegs.ADCSOC2CTL.bit.ACQPS  = acqps;             //  sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read inverter output current phase c
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 3;                  //  ADC D SOC3 will convert pin D3
    AdcdRegs.ADCSOC3CTL.bit.ACQPS  = acqps;             //  sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read inverter output current phase b
    AdcdRegs.ADCSOC4CTL.bit.CHSEL = 4;                  //  ADC D SOC4 will convert pin D4
    AdcdRegs.ADCSOC4CTL.bit.ACQPS  = acqps;             //  sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C
    // Read inverter output current phase a
    AdcdRegs.ADCSOC5CTL.bit.CHSEL = 5;                  //  ADC D SOC5 will convert pin D5
    AdcdRegs.ADCSOC5CTL.bit.ACQPS  = acqps;             //  sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC5CTL.bit.TRIGSEL = TRIGSEL_PWM1_SOC; //  trigger on ePWM1 SOCA/C


    // EOC interrupt settings



//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;              //  end of SOC5 on ADC channel A will set INT1 flag, this should be the longest conversion sequence that take most time
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                //  enable INT1 flag on ADC channel A
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //  make sure INT1 flag is cleared

    // DPJ 02/12/2016 : The code lines below are redundant since the settings for ADC A will have a same effect
//  AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 3;              //  end of SOC3 on ADC channel D
//  AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;                //  enable INT1 flag on ADC channel D
//  AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //  make sure INT1 flag is cleared

//    AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 4;              //  end of SOC4 on ADC channel D
//    AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1;                //  enable INT2 flag on ADC channel D
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              //  make sure INT2 flag is cleared
//
//    AdcdRegs.ADCINTSEL3N4.bit.INT3SEL = 5;              //  end of SOC5 on ADC channel D
//    AdcdRegs.ADCINTSEL3N4.bit.INT3E = 1;                //  enable INT3 flag on ADC channel D
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;              //  make sure INT3 flag is cleared

//    AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 4;              //  end of SOC4 on ADC channel D
//    AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1;                //  enable INT2 flag on ADC channel D
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              //  make sure INT2 flag is cleared


    // CASE A
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 4;              //  end of SOC5 on ADC channel D will set INT1 flag, this should be the longest conversion sequence that take most time
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;                //  enable INT1 flag on ADC channel D
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //  make sure INT1 flag is cleared



    // CASE D

//    AdcdRegs.ADCINTSEL3N4.bit.INT4SEL = 4;
//    AdcdRegs.ADCINTSEL3N4.bit.INT4E = 1;
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;


    EDIS;

    // Enable ADCINT in PIE


//    PieCtrlRegs.PIEIER10.bit.INTx13 = 1; // ADC D
//    PieCtrlRegs.PIEIER10.bit.INTx14 = 1; // ADC D
//    PieCtrlRegs.PIEIER10.bit.INTx15 = 1; // ADC D
//  PieCtrlRegs.PIEIER10.bit.INTx6 = 1; // ADC D



    // CASE A
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  // ADC INT 1.6 D1
    IER |= M_INT1;


    // CASE D
//    PieCtrlRegs.PIEIER10.bit.INTx16 = 1; // ADC INT D4
//    IER |= M_INT10;

}

interrupt void adc_isr() {

    GPIO_WritePin(TEST_PIN, 1);


//  //Read all ADC results into variables

    adc_read.ic = adc_calibra_params.scale_ic * ((ADCCORE_V_COUNT_SCALE * (float)AdcaResultRegs.ADCRESULT14) - adc_calibra_params.offset_ic);   // ADCIN14
    c3 = adc_calibra_params.scale_vc * ((ADCCORE_V_COUNT_SCALE * (float)AdccResultRegs.ADCRESULT3) - adc_calibra_params.offset_vc);    // ADCINC3

    adc_read.ib = adc_calibra_params.scale_ib * ((ADCCORE_V_COUNT_SCALE * (float)AdcbResultRegs.ADCRESULT3) - adc_calibra_params.offset_ib);    // ADCINB3
    a3 = adc_calibra_params.scale_vb * ((ADCCORE_V_COUNT_SCALE * (float)AdcaResultRegs.ADCRESULT3) - adc_calibra_params.offset_vb);    // ADCINA3

    adc_read.ia = adc_calibra_params.scale_ia * ((ADCCORE_V_COUNT_SCALE * (float)AdccResultRegs.ADCRESULT2) - adc_calibra_params.offset_ia);    // ADCINC2
    b2 = adc_calibra_params.scale_va * ((ADCCORE_V_COUNT_SCALE * (float)AdcbResultRegs.ADCRESULT2) - adc_calibra_params.offset_va);    // ADCINB2

    adc_read.idc = ADCCORE_IDC_SCALE * ((ADCCORE_V_COUNT_SCALE * (float)AdcaResultRegs.ADCRESULT2));  //  ADCINA2
    adc_read.vdc = ADCCORE_VDC_SCALE * ((float)AdcaResultRegs.ADCRESULT0);  //  ADCINA0

    adc_read.vdrift = (c3 + a3 + b2) / 3.0;

    adc_read.va = b2 - adc_read.vdrift;
    adc_read.vb = a3 - adc_read.vdrift;
    adc_read.vc = c3 - adc_read.vdrift;

//    c4 = ADCCORE_V_COUNT_SCALE * (float)((float)AdccResultRegs.ADCRESULT4);
//    b4 = ADCCORE_V_COUNT_SCALE * (float)((float)AdcbResultRegs.ADCRESULT4);
//    a4 = ADCCORE_V_COUNT_SCALE * (float)((float)AdcaResultRegs.ADCRESULT4);
//    a1 = ADCCORE_V_COUNT_SCALE * (float)((float)AdcaResultRegs.ADCRESULT1);


    // Normalize voltage and current measurements
    ci.a = adc_norm.ia = adc_read.ia;
    ci.b = adc_norm.ib = adc_read.ib;
    ci.c = adc_norm.ic = adc_read.ic;

    cv.a = adc_norm.va = VOLT_SCALE * adc_read.va;
    cv.b = adc_norm.vb = VOLT_SCALE * adc_read.vb;
    cv.c = adc_norm.vc = VOLT_SCALE * adc_read.vc;

    //  Clarke transformation
    CLARKE_F_FUNC(&cv);
    CLARKE_F_FUNC(&ci);

    smopos.Valpha = cv.alpha;
    smopos.Vbeta = cv.beta;
    pv.alpha = cv.alpha;
    pv.beta = cv.beta;

    smopos.Ialpha = ci.alpha;
    smopos.Ibeta = ci.beta;
    pi.alpha = ci.alpha;
    pi.beta = ci.beta;

    smo(&smopos);

    // Alpha
    smo_a.in = __cos(smopos.Theta); // /*wrotor * k*/

    smo_a.wo2 = smo_a.wo * smo_a.wo;
    smo_a.e_k = smo_a.in - smo_a.a_k_1 - smo_a.a3_k_1 - smo_a.a5_k_1;
    smo_a.a_k = smo_a.a_k_1 + smo_a.T * (smo_a.wo_k_1 * (smo_a.K1 * smo_a.e_k_1 - smo_a.b_k_1));
    smo_a.b_k = smo_a.b_k_1 + smo_a.T * (smo_a.wo_k_1 * smo_a.a_k);
    smo_a.a3_k = smo_a.a3_k_1 + smo_a.T * 3.0 * smo_a.wo_k_1 * (smo_a.K3 * smo_a.e_k_1 - smo_a.b3_k_1);
    smo_a.b3_k = smo_a.b3_k_1 + smo_a.T * 3.0 * smo_a.wo_k_1 * smo_a.a3_k;
    smo_a.a5_k = smo_a.a5_k_1 + smo_a.T * 5.0 * smo_a.wo_k_1 * (smo_a.K5 * smo_a.e_k_1 - smo_a.b5_k_1);
    smo_a.b5_k = smo_a.b5_k_1 + smo_a.T * 5.0 * smo_a.wo_k_1 * smo_a.a5_k;

    smo_a.ef_k =  -smo_a.G * smo_a.e_k *  smo_a.b_k;
    smo_a.Wo_k = smo_a.Wo_k_1 + smo_a.T * (smo_a.ef_k_1 * smo_a.kg_k_1);

    if (smo_a.Wo_k > SYNCHRO_BOUNDARY) {

        smo_a.Wo_k  = SYNCHRO_BOUNDARY;

    }

    if (smo_a.Wo_k < -SYNCHRO_BOUNDARY) {

        smo_a.Wo_k  = -SYNCHRO_BOUNDARY;

    }


    smo_a.vs_k = (smo_a.a_k * smo_a.a_k + smo_a.b_k * smo_a.b_k);

    if (smo_a.vs_k < 1e-5) {

        smo_a.vs_k = 1e-5;
    }

    smo_a.wo_k = smo_a.wc + smo_a.Wo_k;
    smo_a.kg_k = (smo_a.Kg * smo_a.wo_k) / (smo_a.vs_k);

    // Update and store samples
    smo_a.ef_k_1 = smo_a.ef_k;
    smo_a.kg_k_1 = smo_a.kg_k;
    smo_a.wo_k_1 = smo_a.wo_k;
    smo_a.Wo_k_1 = smo_a.Wo_k;
    smo_a.e_k_2 = smo_a.e_k_1;
    smo_a.e_k_1 = smo_a.e_k;
    smo_a.a_k_1 = smo_a.a_k;
    smo_a.b_k_1 = smo_a.b_k;
    smo_a.a3_k_1 = smo_a.a3_k;
    smo_a.b3_k_1 = smo_a.b3_k;
    smo_a.a5_k_1 = smo_a.a5_k;
    smo_a.b5_k_1 = smo_a.b5_k;

    // Beta
    smo_b.in = __sin(smopos.Theta); // /*wrotor * k*/

    smo_b.wo2 = smo_b.wo * smo_b.wo;
    smo_b.e_k = smo_b.in - smo_b.a_k_1 - smo_b.a3_k_1 - smo_b.a5_k_1;
    smo_b.a_k = smo_b.a_k_1 + smo_b.T * (smo_b.wo_k_1 * (smo_b.K1 * smo_b.e_k_1 - smo_b.b_k_1));
    smo_b.b_k = smo_b.b_k_1 +  smo_b.T * (smo_b.wo_k_1 * smo_b.a_k);
    smo_b.a3_k = smo_b.a3_k_1 + smo_b.T * 3.0 * smo_b.wo_k_1 * (smo_b.K3 * smo_b.e_k_1 - smo_b.b3_k_1);
    smo_b.b3_k = smo_b.b3_k_1 +  smo_b.T * 3.0 * smo_b.wo_k_1 * smo_b.a3_k;
    smo_b.a5_k = smo_b.a5_k_1 + smo_b.T * 5.0 * smo_b.wo_k_1 * (smo_b.K5 * smo_b.e_k_1 - smo_b.b5_k_1);
    smo_b.b5_k = smo_b.b5_k_1 +  smo_b.T * 5.0 * smo_b.wo_k_1 * smo_b.a5_k;

    smo_b.ef_k =  -smo_b.G * smo_b.e_k *  smo_b.b_k;
    smo_b.Wo_k = smo_b.Wo_k_1 + smo_b.T * (smo_b.ef_k_1 * smo_b.kg_k_1);

    if (smo_b.Wo_k > SYNCHRO_BOUNDARY) {

        smo_b.Wo_k  = SYNCHRO_BOUNDARY;

    }

    if (smo_b.Wo_k < -SYNCHRO_BOUNDARY) {

        smo_b.Wo_k  = -SYNCHRO_BOUNDARY;

    }

    smo_b.vs_k = (smo_b.a_k * smo_b.a_k + smo_b.b_k * smo_b.b_k);

    if (smo_b.vs_k < 1e-5) {

        smo_b.vs_k = 1e-5;
    }

    smo_b.wo_k = smo_b.wc + smo_b.Wo_k;
    smo_b.kg_k = (smo_b.Kg * smo_b.wo_k) / (smo_b.vs_k);

    // Update and store samples
    smo_b.ef_k_1 = smo_b.ef_k;
    smo_b.kg_k_1 = smo_b.kg_k;
    smo_b.wo_k_1 = smo_b.wo_k;
    smo_b.Wo_k_1 = smo_b.Wo_k;
    smo_b.e_k_2 = smo_b.e_k_1;
    smo_b.e_k_1 = smo_b.e_k;
    smo_b.a_k_1 = smo_b.a_k;
    smo_b.b_k_1 = smo_b.b_k;
    smo_b.a3_k_1 = smo_b.a3_k;
    smo_b.b3_k_1 = smo_b.b3_k;
    smo_b.a5_k_1 = smo_b.a5_k;
    smo_b.b5_k_1 = smo_b.b5_k;

    tempAngle = __atan2(smo_b.a_k, smo_a.a_k);
    tempAngleE = tempAngle; //__atan2(__sin(NUMBER_OF_PP * tempAngle), __cos(NUMBER_OF_PP * tempAngle));

    // Positive and negative sequence
    // xa_pos = 0.5 * ( self.list_of_voltage_resonatorsA[0].xa_t - self.list_of_voltage_resonatorsB[0].xb_t)
    xa_pos = 0.5 * (smo_a.a_k - smo_b.b_k);

    // xb_pos = 0.5 * ( self.list_of_voltage_resonatorsA[0].xb_t + self.list_of_voltage_resonatorsB[0].xa_t)
    xb_pos = 0.5 * (smo_a.b_k + smo_b.a_k);

    x_pos = __sqrt(xa_pos*xa_pos + xb_pos*xb_pos);

    // xa_neg = 0.5 * ( self.list_of_voltage_resonatorsA[0].xa_t + self.list_of_voltage_resonatorsB[0].xb_t)
    xa_neg = 0.5 * (smo_a.a_k + smo_b.b_k);

    // xb_neg = 0.5 * (-self.list_of_voltage_resonatorsA[0].xb_t + self.list_of_voltage_resonatorsB[0].xa_t)
    xb_neg = 0.5 * (smo_a.b_k - smo_b.a_k);

    x_neg = __sqrt(xa_neg*xa_neg + xb_neg*xb_neg);

    // Determine the speed sign
    signSpeed = x_pos < x_neg ? 1.0 : -1.0;


    pv.sin = __sin(tempAngleE);
    pv.cos = __cos(tempAngleE);
    pi.sin = __sin(tempAngleE);
    pi.cos = __cos(tempAngleE);

    pv.zero = cv.zero;
    pi.zero = ci.zero;

    PARK_F_FUNC(&pv);
    PARK_F_FUNC(&pi);

    // Torque estimation
    torque_est = (1.0 - ft) * torque_est_1 + ft * FLUX_M * pi.q;
    torque_est_1 = torque_est;

    // Current controller gains
    dahlin_info.K = dahlin_info.Kinv *  adc_read.vdc;
    dahlin_info.Kp = dahlin_info.tempExpA / (dahlin_info.K * dahlin_info.tempExpB * (1.0 + dahlin_info.N * dahlin_info.tempExpA)) ;
    dahlin_info.Ki = dahlin_info.Kp * dahlin_info.tempExpB;



    // State machine
    switch(control_state_info) {

        // Calibration
        case CALIBRA:

            // Go directly to torque control
            control_state_info = SYNCHRO; //RUN; //

        break;

        // Synchronize
        case SYNCHRO:

            // Call open loop controller

                Nrotor = (ISR_FREQUENCY) / frotor;


                wrotor = (2.0 * PI) / Nrotor;

                k++;
                k %= (int) Nrotor;




                j++;
                j%=(int)100000;

                if (j == 0) {

                    switch (state) {

                    case 0:
                        modul = 0.2;
                        state = 1;
                        break;

                    case 1:
                        modul = 0.2;
                        state = 2;
                        break;

                    case 2:
                        modul = 0.2;
                        state = 3;
                        break;

                    case 3:
                        modul = 0.2;
                        state = 0;
                        break;
                    }

                }

                temp  = modul * __sin(wrotor * k); // TMU instructions
                temp1 = modul * __cos(wrotor * k); // TMU instructions


                svgn.Ualpha = temp1;
                svgn.Ubeta = temp;

                svgen(&svgn);

                EPwm1Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Tc + 1.0))) + 1;
                EPwm2Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Tb + 1.0))) + 1;
                EPwm3Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Ta + 1.0))) + 1;

                // Debugging
//                tempTheta = PI_INV * smopos.Theta;
//                EPwm4Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( tempTheta + 1.0))) + 1;

//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( pv.__relaxed_sin + 1.0))) + 1;
//                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * (  pv.__relaxed_cos  + 1.0))) + 1;



                // DPJ 13/10/2020 : Test iPark. Remove when finished
                ipv.d = pv.d; // 0.5; // idPID.term.Out; //
                ipv.q = pv.q; // 0.5; // iqPID.term.Out; //
                ipv.z = pv.zero;
                ipv.sin = pv.sin; // __sin(smopos.Theta); //
                ipv.cos = pv.cos; // __cos(smopos.Theta); //


                iPARK_F_FUNC(&ipv);

//                smo_a.in = ipv.__relaxed_cos;
//                ResonantIntg(&smo_a);
//
//                smo_b.in = ipv.__relaxed_sin;
//                ResonantIntg(&smo_b);


//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( (PI_INV * tempAngleE) + 1.0))) + 1;
//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( __sin(smopos.Theta) + 1.0))) + 1;
//                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( __cos(smopos.Theta) + 1.0))) + 1;

//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( ipv.beta + 1.0))) + 1;
//                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( ipv.alpha + 1.0))) + 1;


                // DPJ 15/10/2020: Stator flux
                EPwm4Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( (cv.alpha) + 1.0))) + 1;
//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( (cv.beta / VDC_MAX) + 1.0))) + 1;
                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( (ipv.alpha) + 1.0))) + 1;

                // DPJ 15/10/2020: Rotor flux
//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( flux_r.a + 1.0))) + 1;
//                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( flux_r.b + 1.0))) + 1;


                // DPJ 15/10/2020: Rotor flux estimation
//                EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( emfa_r.a_k + 1.0))) + 1;
//                EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( emfb_r.a_k + 1.0))) + 1;


                // Change the state to RUN after 5sec
                state_change_5sec++;
                if (state_change_5sec > STATE_CHNAGE_5SEC) {

                    state_change_5sec = 0;
//                    control_state_info = RUN;

                }


        break;

        // Run closed loop
        case RUN:

            // Speed control
            state_change_5sec++;
            state_change_5sec%=VDC_CTRL;

            if (state_change_5sec == 0) {

#if 1
                ramp_k = 0.99 * ramp_k_1 + 0.01 * wRefRPM;
                torquePID.term.Ref =  ramp_k;
                ramp_k_1 = ramp_k;

                torquePID.term.Fbk = smo_a.wo_k;

                torquePID.param.Kp = KpoS;
                torquePID.param.Ki = KioS * DELTA_T;

                PID_GRANDO_F_FUNC(&torquePID);

#else

                pidS.Kp = KpoS;
                pidS.Ki = KioS;
                pidS.Umax = IDC_MAX;
                pidS.Umin = -IDC_MAX;
                pidS.yref =  wRefRPM;
                pidS.y = smo_a.wo_k;
                pid(&pidS);
#endif
            }

            // Current control
#if 1
            idPID.term.Ref = 0.0;
            idPID.term.Fbk = pi.d;
            idPID.param.Umax = 0.95 * adc_read.vdc;
            idPID.param.Umin = -0.95 * adc_read.vdc;
            idPID.param.Kp = KpoD;
            idPID.param.Ki = KioD * DELTA_T;
            PID_GRANDO_F_FUNC(&idPID);

            iqPID.term.Ref = torquePID.term.Out;
            iqPID.term.Fbk = pi.q;
            iqPID.param.Umax = 0.95 * adc_read.vdc;
            iqPID.param.Umin = -0.95 * adc_read.vdc;
            iqPID.param.Kp = KpoQ;
            iqPID.param.Ki = KioQ * DELTA_T; //
            PID_GRANDO_F_FUNC(&iqPID);

#else

            // id-controller
            pidD.Kp = KpoD;
            pidD.Ki = KioD;
            pidD.Umax =  0.95 * adc_read.vdc;
            pidD.Umin = -0.95 * adc_read.vdc;
            pidD.yref = 0.0;
            pidD.y = pi.d;
            pid(&pidD);

            // iq-controller
            pidQ.Kp = KpoQ;
            pidQ.Ki = KioQ;
            pidQ.Umax =  0.95 * adc_read.vdc;
            pidQ.Umin = -0.95 * adc_read.vdc;
            pidQ.yref = pidS.u_k;
            pidQ.y = pi.q;
            pid(&pidQ);
#endif
            // Inverse Park

#if 1
            ipv.d = idPID.term.Out - (motor_params.Ls * pi.q * smo_b.wo_k);
            ipv.q = iqPID.term.Out + (motor_params.Ls * pi.d * smo_b.wo_k);

#else

            ipv.d = pidD.u_k; // /*idPID.term.Out*/ - (motor_params.Ls * pi.q * smo_b.wo_k);
            ipv.q = pidQ.u_k; // /*iqPID.term.Out*/ + (motor_params.Ls * pi.d * smo_b.wo_k);

#endif

            ipv.z = 0.0;            // pv.zero;

            ipv.sin = __sin(tempAngleE);
            ipv.cos = __cos(tempAngleE);

            iPARK_F_FUNC(&ipv);

            // SVM control signal

            svgn.Ualpha = ipv.alpha / adc_read.vdc;
            svgn.Ubeta = ipv.beta / adc_read.vdc;

            svgen(&svgn);

            EPwm1Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Tc + 1.0))) + 1;
            EPwm2Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Tb + 1.0))) + 1;
            EPwm3Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Ta + 1.0))) + 1;


//            tempTheta = PI_INV * smopos.Theta;
//            EPwm4Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( tempTheta + 1.0))) + 1;
//            EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( (PI_INV * tempAngleE) + 1.0))) + 1;


            EPwm4Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Ualpha + 1.0))) + 1;
            EPwm5Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( svgn.Ubeta + 1.0))) + 1;
            EPwm6Regs.CMPA.bit.CMPA = (Uint16) ((float) (PWM_PERIOD) * ( 0.5 * ( 10.0 * (torque_est) + 1.0))) + 1;

        break;

    }

    GPIO_WritePin(TEST_PIN, 0);

    // CASE A
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear D1 INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // CASE D
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT4 = 1; // clear INT4 flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;


}





































