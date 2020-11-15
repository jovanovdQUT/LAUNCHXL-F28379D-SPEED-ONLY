/*
 * filter.h
 *
 *  Created on: 28 May 2018
 *      Author: Worker
 */

#ifndef HEADERS_FILTER_H_
#define HEADERS_FILTER_H_


typedef struct RESONANT {

    float in;
    float K1;
    float K2;
    float K3;
    float K4;
    float K5;
    float K6;
    float K7;

    float G;        // Phase detector gain
    float Kg;       // Frequency adaptation gain

    float e_k;
    float e_k_1;
    float e_k_2;

    float ef_k;     // Phase detector - gained error signal
    float ef_k_1;   // Phase detector - gained error signal (a previous value)

    float a_k;
    float a_k_1;
    float b_k;
    float b_k_1;

    float a2_k;
    float a2_k_1;
    float b2_k;
    float b2_k_1;

    float a3_k;
    float a3_k_1;
    float b3_k;
    float b3_k_1;

    float a4_k;
    float a4_k_1;
    float b4_k;
    float b4_k_1;

    float a5_k;
    float a5_k_1;
    float b5_k;
    float b5_k_1;

    float a6_k;
    float a6_k_1;
    float b6_k;
    float b6_k_1;

    float a7_k;
    float a7_k_1;
    float b7_k;
    float b7_k_1;

    float T;
    float wo_k;
    float wo_k_1;

    float kg_k;
    float kg_k_1;

    float wo;
    float wo2;
    float wc;

    float Wo_k;
    float Wo_k_1;

    float vs_k;

} resonant_t;

typedef struct FLL_INFO {

    float T;        // Sample period
    float G;        // Phase detector gain
    float Kg;       // Frequency adaptation gain

    float a_e_k;    // Phase detector - alpha input
    float a_vb_k;   // Phase detector - alpha input
    float b_e_k;    // Phase detector - beta input
    float b_vb_k;   // Phase detector - beta input

    float e_k;      // Phase detector - a combined error signal

    float ef_k;     // Phase detector - gained error signal
    float ef_k_1;   // Phase detector - gained error signal (a previous value)

    float a_pos;    // Positive sequence
    float b_pos;    // Positive sequence - qudrature

    float a;        // Alpha
    float qa;       // Alpha quadrature
    float b;        // Beta
    float qb;       // Beta quadrature

    float kg_k;
    float kg_k_1;

//    float wo;

    float Wo_k;
    float Wo_k_1;

    float vs_k;

    float wc;
    float wo_k;
    float w;
//    float wo_k_1;
//    float wo2;

} fll_info_t;

typedef struct BANDPASS {

    float r;
    float cosfi;
    float y_k;
    float y_k_1;
    float y_k_2;
    float u_k;
    float u_k_1;
    float u_k_2;


} bandpass_t;

typedef struct NOTCH_FILTER_INFO {

    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    float out;
    float out_1;
    float out_2;
    float in;
    float in_1;
    float in_2;

} Notch_Filter_Info_t;

typedef struct LP_FILTER_INFO {

    float in;
    float out_k;
    float out_k_1;
    float param;

} lp_filter_t;

typedef struct SIMPLE_INTEGRATOR {

    float out_k;
    float out_k_1;
    float in_k;
    float T;
} sig_t;

void FllInit(fll_info_t*, float, float, float);

void Fll(fll_info_t*);

void ResonantIntgInit(resonant_t*, float, float, float, float, float, float, float, float, float);

void ResonantIntg(resonant_t*);

void BandpassInit(bandpass_t*, float, float);

void Bandpass(bandpass_t*);

void NotchFilterInit(Notch_Filter_Info_t* restrict, float, float, float, float, float);

void NotchFilter(Notch_Filter_Info_t*);

void LpInit(lp_filter_t *, float, float);

void LpFilter(lp_filter_t *);

void SimpleIntgInit(sig_t*, float);

void SimpleIntg(sig_t*);

#endif /* HEADERS_FILTER_H_ */
