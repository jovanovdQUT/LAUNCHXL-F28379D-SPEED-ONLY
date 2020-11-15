/*
 * control.h
 *
 *  Created on: 6 Mar. 2020
 *      Author: Dejan Jovanovic
 */

#ifndef PROJECT_HEADERS_CONTROL_H_
#define PROJECT_HEADERS_CONTROL_H_

#define EXP(x) ((1.0) + (x) + (0.5)*(x)*(x) + (0.1667)*(x)*(x)*(x))

#define SMOPOS_CONST_DEFAULTS {0,0,0,0,0,0,0,}

typedef enum {CALIBRA, SYNCHRO, RUN, ERROR} Control_State_t;
typedef enum {SYNCHRO_OK, SYNCHRO_FAIL} Synchro_State_t;


typedef struct SYNCHRO_INFO {

    Synchro_State_t state;
    Uint32 counter;
    float synchro_time;
    float fo_min;
    float fo_max;

} Synchro_Info_t;

typedef struct  { float  Ualpha;          // Input: reference alpha-axis phase voltage
                  float  Ubeta;           // Input: reference beta-axis phase voltage
                  float  Ta;              // Output: reference phase-a switching function
                  float  Tb;              // Output: reference phase-b switching function
                  float  Tc;              // Output: reference phase-c switching function
                  float  tmp1;            // Variable: temp variable
                  float  tmp2;            // Variable: temp variable
                  float  tmp3;            // Variable: temp variable
                  Uint16 VecSector;       // Space vector sector
} Svgen_t;

//
//typedef struct  { float32  Rs;              // Input: Stator resistance (ohm)
//                  float32  Ls;              // Input: Stator inductance (H)
//                  float32  Ib;              // Input: Base phase current (amp)
//                  float32  Vb;              // Input: Base phase voltage (volt)
//                  float32  Ts;              // Input: Sampling period in sec
//                  float32  Fsmopos;         // Output: constant using in observed current calculation
//                  float32  Gsmopos;         // Output: constant using in observed current calculation
//
//                } SMOPOS_CONST;


typedef struct {  float  Valpha;      // Input: Stationary alfa-axis stator voltage
                  float  Ealpha;      // Variable: Stationary alfa-axis back EMF
                  float  Zalpha;      // Output: Stationary alfa-axis sliding control
                  float  Gsmopos;     // Parameter: Motor dependent control gain
                  float  EstIalpha;   // Variable: Estimated stationary alfa-axis stator current
                  float  Fsmopos;     // Parameter: Motor dependent plant matrix
                  float  Vbeta;       // Input: Stationary beta-axis stator voltage
                  float  Ebeta;       // Variable: Stationary beta-axis back EMF
                  float  Zbeta;       // Output: Stationary beta-axis sliding control
                  float  EstIbeta;    // Variable: Estimated stationary beta-axis stator current
                  float  Ialpha;      // Input: Stationary alfa-axis stator current
                  float  IalphaError; // Variable: Stationary alfa-axis current error
                  float  Kslide;      // Parameter: Sliding control gain
                  float  Ibeta;       // Input: Stationary beta-axis stator current
                  float  IbetaError;  // Variable: Stationary beta-axis current error
                  float  Kslf;        // Parameter: Sliding control filter gain
                  float  Theta;       // Output: Compensated rotor angle
                  float  E0;          // Parameter: 0.5
                  float  Rs;              // Input: Stator resistance (ohm)
                  float  Ls;              // Input: Stator inductance (H)
                  float  Ib;              // Input: Base phase current (amp)
                  float  Vb;              // Input: Base phase voltage (volt)
                  float  Ts;              // Input: Sampling period in sec

                 } SMOPOS_F;


 typedef struct {
       float EstimatedTheta;      // Input: Electrical angle (pu)
       float OldEstimatedTheta;   // History: Electrical angle at previous step (pu)
       float EstimatedSpeed;      // Output: Estimated speed in per-unit  (pu)
       float BaseRpm;          // Parameter: Base speed in rpm (Q0) - independently with global Q
       float K1;                // Parameter: Constant for differentiator (Q21) - independently with global Q
       float K2;                  // Parameter: Constant for low-pass filter (pu)
       float K3;                  // Parameter: Constant for low-pass filter (pu)
       float EstimatedSpeedRpm; // Output : Estimated speed in rpm  (Q0) - independently with global Q
       float Temp;                // Variable : Temp variable
 } SPEED_ESTIMATION_F;      // Data type created


 typedef struct {
     float yref;
     float y;
     float e_k;
     float e_k_1;
     float u_k;
     float u_k_1;
     float Kp;
     float Ki;
     float temp;
     float vdc;
 } PID;

 typedef struct MOTOR_PARAMS {

     float   Rs;
     float   Ls;
     float   pole_pairs;

 } motor_params_t;


 typedef struct PD_INFO {

     float xa;
     float xb;

     float xan;
     float xbn;
     float mag;

     float Kp;
     float Ti;
     float Kc;
     float Ts;
     float Ki;
     float e_k;
     float e_k_1;
     float p_k;
     float i_k;
     float i_k_1;
     float ipresat;
     float isat;
     float reg;
     float theta_k;
     float theta_k_1;
     float w_k;
     float w_k_1;

 } pd_info_t;


 typedef struct EMF_INFO {

     float a;
     float b;

 } emf_info_t;


 typedef struct FLUX_INFO {

     float a;
     float b;

 } flux_info_t;


typedef struct DAHLIN_INFO {

     float lambda;
     float Ts;
     float To;
     float N;
     float K;
     float Kp;
     float Ki;
     float Kinv;
     float tempExpA;
     float tempExpB;

 } dahlin_info_t;

void svgen(Svgen_t*);

void InitControl(void);
void InitSynchroInfo(void);
void WakeUpDrv(void);

void pdinit(pd_info_t*, float);
void pd(pd_info_t*);

void smo_init(SMOPOS_F*, float, float, float, float, float);
void smo(SMOPOS_F*);

void se_init(SPEED_ESTIMATION_F*, float, float , float , float);
void se(SPEED_ESTIMATION_F*);

void pid_init(PID*, float, float);
void pid(PID*);


#endif /* PROJECT_HEADERS_CONTROL_H_ */
