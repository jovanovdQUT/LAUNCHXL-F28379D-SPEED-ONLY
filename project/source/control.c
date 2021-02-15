/*
 * control.c
 *
 *  Created on: 6 Mar. 2020
 *      Author: Dejan Jovanovic
 */
#include "F28x_Project.h"     // Device Headerfile and Examples Include File

#include <math.h>

#include "Solar_F.h"
#include "settings.h"
#include "control.h"
#include "InstaSPIN_BLDC_Lib.h"
#include "filter.h"
#include "PQ.h"


#define MATH_TYPE   FLOAT_MATH

SPLL_1ph_SOGI_F spll1 = {0};

Control_State_t control_state_info = {CALIBRA};

Synchro_Info_t synchro_info;

CLARKE_F cv, ci;
PARK_F pv, pi;
iPARK_F ipv;

SMOPOS_F smopos;
SPEED_ESTIMATION_F speedest;

PID_GRANDO_F_CONTROLLER torquePID, idPID, iqPID;
PID pidD, pidQ, pidS;

motor_params_t motor_params = {0};
fll_info_t fll, fll_r;
resonant_t emfa, emfb, emfa_r, emfb_r;
pd_info_t pd_info, pd_info_r;
emf_info_t emf;
flux_info_t flux_r, flux_s;
synchro_3f_info_t synchro_3f;
sample_info_t sample_info;

dahlin_info_t dahlin_info;

resonant_t smo_a={0}, smo_b={0};

Uint32 state_change_5sec = 0, vdc_ctrl_cnt = 0;

float wRefRPM = 80.0 * TWO_PI;;
float tempConst = 0.0;


    // Motor Parameters
float Rs = 2.0, Ls = 0.0023, Ib = IDC_MAX, Vb = VDC_MAX;
float fb = (2000.0*2.0*PI)/60.0;
float BaseRpm = ((120.0*2000.0*2*PI)/60.0)/10.0, fc = (2000.0*2.0*PI/60.0)*10.0;

float Kp = 0.5, Ki = 0.001;
float Fo = 1.0, G = 1.0, D = 0.1,  K = 1.0;

void InitControl(void) {

    // Initialize PLL
    SPLL_1ph_SOGI_F_init((GRID_FREQ), ((float) ((1.0) / (ISR_FREQUENCY))), &spll1);
    SPLL_1ph_SOGI_F_coeff_update(((float) ((1.0) / (ISR_FREQUENCY))), (float) ((2.0) * (PI) * (GRID_FREQ)),  &spll1);

    // Synchronization info initialization
    InitSynchroInfo();

    // Bring the BOOSTXL-DRV8305EVM out of its low power sleep mode
//    WakeUpDrv();



    // Initialize Clarke
    CLARKE_F_init(&cv);
    CLARKE_F_init(&ci);

    // Initialize Park
    PARK_F_init(&pv);
    PARK_F_init(&pi);

    iPARK_F_init(&ipv);

    // SMO initialization
    smo_init(&smopos, Rs, Ls, Ib, Vb, DELTA_T);

    // Speed estimation
    se_init(&speedest, BaseRpm, DELTA_T, fb, fc);

    // PID initialization
    PID_GRANDO_F_init(&torquePID);
    torquePID.param.Kp = 0.075;
    torquePID.param.Ki = 1e-6; // 0.0 * 0.1;
    torquePID.param.Umax =  IDC_MAX;
    torquePID.param.Umin = -IDC_MAX;


    PID_GRANDO_F_init(&idPID);
    idPID.param.Kp = 1.0; // 0.075;
    idPID.param.Ki = 0.0; // 1e-6; // 0.0 * 0.1;

    PID_GRANDO_F_init(&iqPID);
    iqPID.param.Kp = 1.0; // 0.1; // 0.075;
    iqPID.param.Ki = 0.0; // 1e-6; //  0.0 * 0.1;

    state_change_5sec = 0;
    vdc_ctrl_cnt = 0;

    // Positional PI controllers
    pid_init(&pidS, Kp, Ki, (float) DELTA_T);

    pid_init(&pidD, Kp, Ki, (float) DELTA_T);
    pid_init(&pidQ, Kp, Ki, (float) DELTA_T);


    // Motor parameters
    motor_params.Rs = Rs;
    motor_params.Ls = Ls;
    motor_params.pole_pairs = 4.0;

    // FLL initialization
    fll.w = 0;
    fll_r.w = 0;

    // Initialize FLL
    tempConst = 1.4142; // 0.3  /*(0.75 * 20.0 * 1.4142)*/;
    FllInit(&fll, (float) DELTA_T, (float) /*30*/ /*0.02*/ 4.6, (float) tempConst);
    FllInit(&fll_r, (float) DELTA_T, (float) /*30*/ /*0.02*/ 4.6, (float) tempConst);

    // Initialize SOG
    tempConst = 1.4142; // 3 /*1.4142*/
    ResonantIntgInit(&emfa, (float) 1.0, (float) DELTA_T, (float) tempConst, (float) 0.05*tempConst, (float) tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst);
    ResonantIntgInit(&emfb, (float) 1.0, (float) DELTA_T, (float) tempConst, (float) 0.05*tempConst, (float) tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst);

    ResonantIntgInit(&emfa_r, (float) 1.0, (float) DELTA_T, (float) tempConst, (float) 0.05*tempConst, (float) tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst);
    ResonantIntgInit(&emfb_r, (float) 1.0, (float) DELTA_T, (float) tempConst, (float) 0.05*tempConst, (float) tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst, (float) 0.05*tempConst, (float) 0.8*tempConst);


//    tempConst = 1.0;
//    ResonantIntgInit(&smo_a, (float) 50.0, (float) DELTA_T, (float) tempConst, (float) 0.00*tempConst, (float) 0.9*tempConst, (float) 0.00*tempConst, (float) 0.0*tempConst, (float) 0.00*tempConst, (float) 0.0*tempConst);
//    ResonantIntgInit(&smo_b, (float) 50.0, (float) DELTA_T, (float) tempConst, (float) 0.00*tempConst, (float) 0.9*tempConst, (float) 0.00*tempConst, (float) 0.0*tempConst, (float) 0.00*tempConst, (float) 0.0*tempConst);


    smo_a.K1 = 1.4142;
    smo_a.K3 = 1.4142;
    smo_a.K5 = 1.4142;
    smo_a.G = 4.6;
    smo_a.Kg = 1.4142;
    smo_a.T = DELTA_T;
    smo_a.wo_k = 2.0 * ((float) TWO_PI) * 1.00;
    smo_a.wo_k_1 = 0.0;
    smo_a.wc = SYNCHRO_RANGE; // ((float) TWO_PI) * 100.00;
    smo_a.wo = 0;
    smo_a.wo2 = 0;
    smo_a.Wo_k = smo_a.Wo_k_1 = 0;
    smo_a.e_k_1 = smo_a.e_k = 0;
    smo_a.ef_k = smo_a.ef_k_1 = 0;
    smo_a.kg_k = smo_a.kg_k_1 = 0;
    smo_a.a_k_1 = smo_a.a_k = 0;
    smo_a.b_k_1 = smo_a.b_k = 0;
    smo_a.a3_k_1 = smo_a.a3_k = 0;
    smo_a.b3_k_1 = smo_a.b3_k = 0;
    smo_a.a5_k_1 = smo_a.a5_k = 0;
    smo_a.b5_k_1 = smo_a.b5_k = 0;

    smo_b.K1 = 1.4142;
    smo_b.K3 = 1.4142;
    smo_b.K5 = 1.4142;
    smo_b.G = 4.6;
    smo_b.Kg =  1.0;
    smo_b.T = DELTA_T;
    smo_b.wo_k = 2.0 * ((float) TWO_PI) * 1.00;
    smo_b.wo_k_1 = 0.0;
    smo_b.wc = SYNCHRO_RANGE; // ((float) TWO_PI) * 100.00;
    smo_b.wo = 0;
    smo_b.wo2 = 0;
    smo_b.Wo_k = smo_b.Wo_k_1 = 0;
    smo_b.e_k_1 = smo_b.e_k = 0;
    smo_b.ef_k = smo_b.ef_k_1 = 0;
    smo_b.kg_k = smo_b.kg_k_1 = 0;
    smo_b.a_k_1 = smo_b.a_k = 0;
    smo_b.b_k_1 = smo_b.b_k = 0;
    smo_b.a3_k_1 = smo_b.a3_k = 0;
    smo_b.b3_k_1 = smo_b.b3_k = 0;
    smo_b.a5_k_1 = smo_b.a5_k = 0;
    smo_b.b5_k_1 = smo_b.b5_k = 0;

    // PD initialization
    pdinit(&pd_info, (float) DELTA_T);
    pdinit(&pd_info_r, (float) DELTA_T);

    // Three-phase synchronisation
    synchro_3f_init(&synchro_3f, (float) DELTA_T, Fo, G, D, K);

    // Dahlin algorithm

    dahlin_info.Kinv = INVSQRT3 / Rs;

    dahlin_info.N = 1.0;
    dahlin_info.To = Ls/ Rs;
    dahlin_info.Ts = DELTA_T;
    dahlin_info.lambda = 1e3; // 20/20ms
    dahlin_info.tempExpA = 1.0 - exp(-dahlin_info.lambda*dahlin_info.Ts);
    dahlin_info.tempExpB = exp(dahlin_info.Ts/dahlin_info.To) - 1.0;

    dahlin_info.K = dahlin_info.Kinv * VDC_MAX;
    dahlin_info.Kp = dahlin_info.tempExpA / (dahlin_info.K * dahlin_info.tempExpB * (1.0 + dahlin_info.N * dahlin_info.tempExpA)) ;
    dahlin_info.Ki = dahlin_info.Kp * dahlin_info.tempExpB;
}

void InitSynchroInfo(void) {

    synchro_info.counter = 0;
    synchro_info.state = SYNCHRO_FAIL;
    synchro_info.fo_min = GRID_FREQ_MIN;
    synchro_info.fo_max = GRID_FREQ_MAX;
    synchro_info.synchro_time = 0.0;
}

void WakeUpDrv(void) {

    GPIO_WritePin(WAKE_A, SET);

//    GPIO_WritePin(WAKE_A, SET);
//    //
//    // Delay for a bit.
//    //
//    DELAY_US(1000);
//
//    GPIO_WritePin(WAKE_A, RESET);
//
//    GPIO_WritePin(WAKE_B, SET);
//    //
//    // Delay for a bit.
//    //
//    DELAY_US(1000);
//
//    GPIO_WritePin(WAKE_B, RESET);

}

void svgen(Svgen_t* v) {

    v->tmp1 = v->Ubeta;
    v->tmp2 = 0.5 * v->Ubeta + 0.8660254 * v->Ualpha;
    v->tmp3 = v->tmp2 - v->tmp1;

    v->VecSector = 3;
    v->VecSector = (v->tmp2 > 0) ? (v->VecSector - 1) : v->VecSector;
    v->VecSector = (v->tmp3 > 0) ? (v->VecSector - 1) : v->VecSector;
    v->VecSector = (v->tmp1 < 0) ? (7 - v->VecSector) : v->VecSector;

    if     (v->VecSector == 1 || v->VecSector == 4)
      {     v->Ta =  v->tmp2;
            v->Tb =  v->tmp1 - v->tmp3;
            v->Tc = -v->tmp2;
      }

    else if(v->VecSector == 2 || v->VecSector == 5)
      {     v->Ta =  v->tmp3 + v->tmp2;
            v->Tb =  v->tmp1;
            v->Tc = -v->tmp1;
      }

    else
      {     v->Ta =  v->tmp3;
            v->Tb = -v->tmp3;
            v->Tc = -(v->tmp1 + v->tmp2);
      }


}



void smo(SMOPOS_F* v) {

//    /*  Sliding mode current observer   */                                                              \
//    v.EstIalpha = _IQmpy(v.Fsmopos,v.EstIalpha) + _IQmpy(v.Gsmopos,(v.Valpha-v.Ealpha-v.Zalpha));       \
//    v.EstIbeta  = _IQmpy(v.Fsmopos,v.EstIbeta)  + _IQmpy(v.Gsmopos,(v.Vbeta -v.Ebeta -v.Zbeta ));       \

    /*  Sliding mode current observer   */
    v->EstIalpha = (v->Fsmopos*v->EstIalpha) + v->Gsmopos*(v->Valpha - v->Ealpha -v->Zalpha);
    v->EstIbeta  = (v->Fsmopos*v->EstIbeta)  + v->Gsmopos*(v->Vbeta  - v->Ebeta - v->Zbeta );





//    /*  Current errors  */                                                                              \
//    v.IalphaError = v.EstIalpha - v.Ialpha;                                                             \
//    v.IbetaError  = v.EstIbeta  - v.Ibeta;


    //    /*  Current errors  */
    v->IalphaError = v->EstIalpha - v->Ialpha;
    v->IbetaError  = v->EstIbeta  - v->Ibeta;





//    /*  Sliding control calculator  */                                                                  \
//    /* v.Zalpha=v.IalphaError*v.Kslide/v.E0) where E0=0.5 here*/                                        \
//    v.Zalpha = _IQmpy(_IQsat(v.IalphaError,v.E0,-v.E0),_IQmpy2(v.Kslide));                              \
//    v.Zbeta  = _IQmpy(_IQsat(v.IbetaError ,v.E0,-v.E0),_IQmpy2(v.Kslide));                              \


    /*  Sliding control calculator  */
    if (v->IalphaError >= v->E0){
        v->IalphaError = v->E0;
    }

    else if (v->IalphaError <= -v->E0) {
    v->IalphaError = -v->E0;
    }

    if (v->IbetaError >= v->E0) {
        v->IbetaError = v->E0;
    }

    else if (v->IbetaError <= -v->E0) {
    v->IbetaError = -v->E0;
    }

    v->Zalpha = 2.0*v->IalphaError*(v->Kslide);
    v->Zbeta  = 2.0*v->IbetaError *(v->Kslide);

//    /*  Sliding control filter -> back EMF calculator   */                                              \
//    v.Ealpha = v.Ealpha + _IQmpy(v.Kslf,(v.Zalpha-v.Ealpha));                                           \
//    v.Ebeta  = v.Ebeta  + _IQmpy(v.Kslf,(v.Zbeta -v.Ebeta));

    /*  Sliding control filter -> back EMF calculator   */
    v->Ealpha = v->Ealpha + v->Kslf*(v->Zalpha-v->Ealpha);
    v->Ebeta  = v->Ebeta  + v->Kslf*(v->Zbeta -v->Ebeta);


//    /*  Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)   */
//    v.Theta = _IQatan2PU(-v.Ealpha,v.Ebeta);


    /*  Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)   */
    v->Theta = atan2f(-v->Ealpha,v->Ebeta);

}



void smo_init(SMOPOS_F* v, float Rs, float Ls, float Ib, float Vb, float Ts) {

    v->Theta = 0;

    v->Rs = Rs;
    v->Ls = Ls;

    v->Ib = Ib;
    v->Vb = Vb;

    v->Ts = Ts;

    v->E0 = 0.5;

    v->Kslf = 0.01;      // ???
    v->Kslide = 0.01;    // ???

    // SMO alpha components
    v->Zalpha = 0;
    v->Ealpha = 0;
    v->EstIalpha = 0;
    v->IalphaError = 0;

    // SMO beta components
    v->Zbeta = 0;
    v->Ebeta = 0;
    v->EstIbeta = 0;
    v->IbetaError = 0;

    // SMO parameters
    v->Fsmopos = exp((-v->Rs / v->Ls)*(v->Ts));
    v->Gsmopos = (v->Vb/v->Ib)*(1.0/v->Rs)*(1.0-v->Fsmopos);



}


void se_init(SPEED_ESTIMATION_F* v, float BaseRpm, float Ts, float fb, float fc) {

    v->EstimatedTheta = 0;      // Input: Electrical angle (pu)
    v->OldEstimatedTheta = 0;   // History: Electrical angle at previous step (pu)
    v->EstimatedSpeed = 0;      // Output: Estimated speed in per-unit  (pu)
    v->BaseRpm = BaseRpm;          // Parameter: Base speed in rpm (Q0) - independently with global Q
    v->K1 = 1.0/(Ts*fb);                // Parameter: Constant for differentiator (Q21) - independently with global Q
    v->K2 = 1.0/(1+Ts*2*PI*fc);                  // Parameter: Constant for low-pass filter (pu)
    v->K3 = Ts*2.0*PI*fc/(1.0+Ts*2.0*PI*fc);                  // Parameter: Constant for low-pass filter (pu)
    v->EstimatedSpeedRpm = 0; // Output : Estimated speed in rpm  (Q0) - independently with global Q
    v->Temp = 0;                // Variable : Temp variable

}


void se(SPEED_ESTIMATION_F* v) {


    /* Synchronous speed computation   */
        v->Temp = v->EstimatedTheta - v->OldEstimatedTheta;
        if (v->Temp < -0.5)
            v->Temp = v->Temp + 1.0;
        else if (v->Temp > 0.5)
            v-> Temp= v->Temp - 1;
        v->Temp = v->K1*v->Temp;

    /* Low-pass filter */
    /* Q21 = GLOBAL_Q*Q21 + GLOBAL_Q*Q21 */
        v->Temp = (v->K2*v->EstimatedSpeed)+(v->K3*v->Temp);

    /* Saturate the output */

        if (v->Temp >= 1.0){
            v->Temp = 1.0;
        }

        else if (v->Temp <= -1.0) {
            v->Temp = -1.0;
        }
        v->EstimatedSpeed = v->Temp;

    /* Update the electrical angle */
        v->OldEstimatedTheta = v->EstimatedTheta;

    /* Change motor speed from pu value to rpm value (GLOBAL_Q -> Q0)*/
    /* Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q*/
        v->EstimatedSpeedRpm = v->BaseRpm*v->EstimatedSpeed;

}


void pid_init(PID* v, float Kp, float Ki, float Ts) {

    v->y = v->yref;

    v->Kp = Kp;
    v->Ki = Ki;

    v->e_k = v->e_k_1 = 0;
    v->temp = 0;
    v->u_k = v->u_k_1 = v->du_k = 0;
    v->vdc = 1.0;

    v->Umax = 1.0;
    v->Umin = -1.0;

    v->T = Ts;

}

void pid(PID* v) {

    v->e_k = v->yref - v->y;

    v->du_k = Kp * (v->e_k - v->e_k_1)  + v->T * v->Ki * v->e_k;

    v->u_k = v->u_k_1 +  v->du_k;

    if (v->u_k > v->Umax) {

        v->u_k = v->Umax;

    }

    if (v->u_k < -v->Umin) {

        v->u_k = -v->Umin;
    }


    v->e_k_1 = v->e_k;

    v->u_k_1 = v->u_k;
}

void pdinit(pd_info_t* pd, float Ts) {

    pd->xa = pd->xan =0;
    pd->xb = pd->xbn =0;

    pd->Kp =  5.0; // 2.2; //10; // 5; // 2.2 * 5; // 222.1603;
    pd->Ti = 0.00822;
    pd->Kc = 0.05; // 5%
    pd->Ts = Ts;
    pd->Ki = ((float) pd->Kp * (float) Ts) / ((float) pd->Ti);
    pd->e_k = pd->e_k_1 = 0;
    pd->p_k = 0;
    pd->i_k = pd->i_k_1 = 0;
    pd->theta_k = pd->theta_k_1 = ((float) TWO_PI);
    pd->w_k = pd->w_k_1 = 1;
    pd->ipresat = pd->isat = 0;
}

void pd(pd_info_t* pd) {

    pd->mag = sqrt(pd->xa * pd->xa + pd->xb * pd->xb);

    if (pd->mag < 1e-3)
        pd->mag = 1e-3;

    pd->xan = pd->xa / pd->mag;
    pd->xbn = pd->xb / pd->mag;

    pd->e_k = (pd->xan * cos(pd->theta_k_1) - pd->xbn * sin(pd->theta_k_1));

    pd->p_k = pd->Kp * pd->e_k;
    pd->i_k = pd->i_k_1 + pd->Ki * pd->e_k_1;

//    pd->ipresat = pd->p_k + pd->i_k;
//
//    if (pd->ipresat > U_MAX) {
//
//        pd->isat = U_MAX;
//    } else if (pd->ipresat < -U_MAX) {
//
//        pd->isat = -U_MAX;
//    }
//
//    pd->i_k = pd->i_k_1 + pd->Ki * pd->e_k_1 + pd->Kc * (pd->isat - pd->ipresat);

    pd->reg = pd->p_k + pd->i_k;
    pd->w_k = pd->reg;
    pd->theta_k = pd->theta_k_1 + pd->Ts * pd->w_k_1;

    if (pd->theta_k > ((float) TWO_PI) /*PI*/) {

//        pd->N = pd->theta_k / ((float) TWO_PI);
//        pd->theta_k = pd->theta_k_1 - ((float) pd->N) * ((float) TWO_PI);

        pd->theta_k = pd->theta_k_1 - ((float) TWO_PI);

    } else if (pd->theta_k < 0) {

        pd->theta_k = pd->theta_k_1 + ((float) TWO_PI);
    }

    pd->e_k_1 = pd->e_k;
    pd->i_k_1 = pd->i_k;
    pd->theta_k_1 = pd->theta_k;
    pd->w_k_1 = pd->w_k;

}
