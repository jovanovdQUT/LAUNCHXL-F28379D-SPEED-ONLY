/*
 * filter.c
 *
 *  Created on: 28 May 2018
 *      Author: Worker
 */

#include <math.h>

#include "settings.h"
#include "filter.h"

void FllInit(fll_info_t* fll, float T, float G, float Kg) {

    fll->T = T;

    fll->G = G;

    fll->Kg = Kg;

    fll->wc = 0; // 10.0; // 2.0 * ((float) TWO_PI) * 100.00;

    fll->wo_k = 2.0 * ((float) TWO_PI) * 1.00;

    fll->Wo_k = fll->Wo_k_1 = 0;

    fll->ef_k = fll->ef_k_1 = 0;

    fll->kg_k = fll->kg_k_1 = 0;

}

void Fll(fll_info_t* fll) {

    // FLL
#if 0  // This is for three phase case

    fll->e_k = 0.5 * ( fll->a_e_k *  fll->qa + fll->b_e_k * fll->qb);

#else

    fll->e_k = fll->a_e_k *  fll->qa;

#endif

    fll->ef_k =  -fll->G * fll->e_k;

    // Wo(i) = Wo(i-1) + Ts * ef(i-1) * kg(i-1);
    fll->Wo_k = fll->Wo_k_1 + fll->T * (fll->ef_k_1 * fll->kg_k_1);

//    if (fll->Wo_k > SYNCHRO_BOUNDARY) {
//
//    fll->Wo_k = SYNCHRO_BOUNDARY;
//    }
//
//    if (fll->Wo_k < -SYNCHRO_BOUNDARY) {
//
//        fll->Wo_k = -SYNCHRO_BOUNDARY;
//
//    }

    // wo(i) = wc + Wo(i);
    fll->wo_k = fll->wc + fll->Wo_k;

    // vs(i) = (va1(i)^2 + vb1(i)^2);

#if 0 // This is for three phase case
    fll->a_pos = 0.5 * (fll->a - fll->qb);

    fll->b_pos = 0.5 * (fll->qa + fll->b);

#else

    fll->a_pos = fll->a;

    fll->b_pos = fll->qa;

#endif

    fll->vs_k = fll->a_pos * fll->a_pos + fll->b_pos * fll->b_pos;

    if (fll->vs_k < 1e-3) {

       fll->vs_k = 1e-3;

    }



    // kg(i) = (Kg * wo(i)) / vs(i);
    fll->kg_k = (fll->Kg * fll->wo_k) / fll->vs_k;

    // Update and store samples
    fll->ef_k_1 = fll->ef_k;

    fll->kg_k_1 = fll->kg_k;

    fll->Wo_k_1 = fll->Wo_k;


}

void ResonantIntgInit(resonant_t* resonant, float wo, float T, float K1, float K2, float K3, float K4, float K5, float K6, float K7) {

    resonant->K1 = K1;    // 1.4142

    resonant->K2 = K2;

    resonant->K3 = K3;

    resonant->K4 = K4;

    resonant->K5 = K5;

    resonant->K6 = K6;

    resonant->K7 = K7;

    resonant->G = 1.0; // 4.6; //G;  //

    resonant->Kg =  1.0; // 0.75 * 20.0 * 1.4142; //Kg; //

    resonant->T = T;

    resonant->wo_k = 2.0 * ((float) TWO_PI) * 1.00;
    resonant->wo_k_1 = 0.0;

    resonant->wc = ((float) TWO_PI) * 50.00;

    resonant->wo = 0;

    resonant->wo2 = 0;

    resonant->Wo_k = resonant->Wo_k_1 = 0;


    resonant->e_k_1 = resonant->e_k = 0;

    resonant->ef_k = resonant->ef_k_1 = 0;

    resonant->kg_k = resonant->kg_k_1 = 0;


    resonant->a_k_1 = resonant->a_k = 0;

    resonant->b_k_1 = resonant->b_k = 0;

    resonant->a2_k_1 = resonant->a2_k = 0;

    resonant->b2_k_1 = resonant->b2_k = 0;

    resonant->a3_k_1 = resonant->a3_k = 0;

    resonant->b3_k_1 = resonant->b3_k = 0;

    resonant->a4_k_1 = resonant->a4_k = 0;

    resonant->b4_k_1 = resonant->b4_k = 0;

    resonant->a5_k_1 = resonant->a5_k = 0;

    resonant->b5_k_1 = resonant->b5_k = 0;

    resonant->a6_k_1 = resonant->a6_k = 0;

    resonant->b6_k_1 = resonant->b6_k = 0;

    resonant->a7_k_1 = resonant->a7_k = 0;

    resonant->b7_k_1 = resonant->b7_k = 0;
}

void ResonantIntg(resonant_t* resonant) {

    resonant->wo2 = resonant->wo * resonant->wo;

    // Input to the resonant integrator
    resonant->e_k = resonant->in - resonant->a_k_1; // - resonant->a2_k_1 - resonant->a3_k_1 - resonant->a4_k_1 - resonant->a5_k - resonant->a6_k_1 - resonant->a7_k;

    // Fundamental frequency

    resonant->a_k = resonant->a_k_1 + resonant->T * (resonant->wo_k_1 * (resonant->K1 * resonant->e_k_1 - resonant->b_k_1));

    resonant->b_k = resonant->b_k_1 +  resonant->T * (resonant->wo_k * resonant->a_k);

//    // Harmonics
//
//    // The 2nd harmonic
//    resonant->a2_k = resonant->a2_k_1 + resonant->T * 2.0 * resonant->wo_k_1 * (resonant->K2 * resonant->e_k_1 - resonant->b2_k_1);
//
//    resonant->b2_k = resonant->b2_k_1 +  resonant->T * 2.0 * resonant->wo_k_1 * resonant->a2_k;
//
//    // The 3rd harmonic
//    resonant->a3_k = resonant->a3_k_1 + resonant->T * 3.0 * resonant->wo_k_1 * (resonant->K3 * resonant->e_k_1 - resonant->b3_k_1);
//
//    resonant->b3_k = resonant->b3_k_1 +  resonant->T * 3.0 * resonant->wo_k_1 * resonant->a3_k;
//
//    // The 4th harmonic
//    resonant->a4_k = resonant->a4_k_1 + resonant->T * 4.0 * resonant->wo_k_1 * (resonant->K4 * resonant->e_k_1 - resonant->b4_k_1);
//
//    resonant->b4_k = resonant->b4_k_1 +  resonant->T * 4.0 * resonant->wo_k_1 * resonant->a4_k;
//
//    // The 5rd harmonic
//    resonant->a5_k = resonant->a5_k_1 + resonant->T * 5.0 * resonant->wo_k_1 * (resonant->K5 * resonant->e_k_1 - resonant->b5_k_1);
//
//    resonant->b5_k = resonant->b5_k_1 +  resonant->T * 5.0 * resonant->wo_k_1 * resonant->a5_k;
//
//    // The 6th harmonic
//    resonant->a6_k = resonant->a6_k_1 + resonant->T * 6.0 * resonant->wo_k_1 * (resonant->K6 * resonant->e_k_1 - resonant->b6_k_1);
//
//    resonant->b6_k = resonant->b6_k_1 +  resonant->T * 6.0 * resonant->wo_k_1 * resonant->a6_k;
//
//    // The 7th harmonic
//    resonant->a7_k = resonant->a7_k_1 + resonant->T * 7.0 * resonant->wo_k_1 * (resonant->K7 * resonant->e_k_1 - resonant->b7_k_1);
//
//    resonant->b7_k = resonant->b7_k_1 +  resonant->T * 7.0 * resonant->wo_k_1 * resonant->a7_k;


    // FLL
    //     ef(i) =  -G * e(i) * vb1(i);
    resonant->ef_k =  -resonant->G * resonant->e_k *  resonant->b_k;

    // Wo(i) = Wo(i-1) + Ts * ef(i-1) * kg(i-1);
    resonant->Wo_k = resonant->Wo_k_1 + resonant->T * (resonant->ef_k_1 * resonant->kg_k_1);

//    if (resonant->Wo_k > SYNCHRO_BOUNDARY) {
//
//        resonant->Wo_k = SYNCHRO_BOUNDARY;
//    }
//
//    if (resonant->Wo_k < -SYNCHRO_BOUNDARY) {
//
//        resonant->Wo_k = -SYNCHRO_BOUNDARY;
//    }

    // vs(i) = (va1(i)^2 + vb1(i)^2);
    resonant->vs_k = __sqrt(resonant->a_k * resonant->a_k + resonant->b_k * resonant->b_k);

    if (resonant->vs_k < 2.0*1e-1) {

        resonant->vs_k = 2.0*1e-1;

    }

    // wo(i) = wc + Wo(i);

    resonant->wo_k = resonant->wc + resonant->Wo_k;

    // kg(i) = (Kg * wo(i)) / vs(i);
    resonant->kg_k = (resonant->Kg * resonant->wo_k) / (resonant->vs_k);

    // Update and store samples
    resonant->ef_k_1 = resonant->ef_k;

    resonant->kg_k_1 = resonant->kg_k;

    resonant->wo_k_1 = resonant->wo_k;

    resonant->Wo_k_1 = resonant->Wo_k;

    resonant->e_k_2 = resonant->e_k_1;

    resonant->e_k_1 = resonant->e_k;

    resonant->a_k_1 = resonant->a_k;

    resonant->b_k_1 = resonant->b_k;

//    resonant->a2_k_1 = resonant->a2_k;
//
//    resonant->b2_k_1 = resonant->b2_k;
//
//    resonant->a3_k_1 = resonant->a3_k;
//
//    resonant->b3_k_1 = resonant->b3_k;
//
//    resonant->a4_k_1 = resonant->a4_k;
//
//    resonant->b4_k_1 = resonant->b4_k;
//
//    resonant->a5_k_1 = resonant->a5_k;
//
//    resonant->b5_k_1 = resonant->b5_k;
//
//    resonant->a6_k_1 = resonant->a6_k;
//
//    resonant->b6_k_1 = resonant->b6_k;
//
//    resonant->a7_k_1 = resonant->a7_k;
//
//    resonant->b7_k_1 = resonant->b7_k;
}

void BandpassInit(bandpass_t* bandpass, float fres, float r) {

    bandpass->cosfi = cos(2.0*((float)TWO_PI)*fres*((float)DELTA_T));

    bandpass->r = r;

    bandpass->u_k = bandpass->u_k_1 = 0;

    bandpass->y_k = bandpass->y_k_1 = bandpass->y_k_2 = 0;



}

void Bandpass(bandpass_t* bandpass){

    bandpass->y_k = 2.0 * bandpass->r * bandpass->cosfi * bandpass->y_k_1 -  (bandpass->r *  bandpass->r) *  bandpass->y_k_2 + bandpass->u_k - bandpass->u_k_2;

    bandpass->u_k_2 = bandpass->u_k_1;

    bandpass->u_k_1 = bandpass->u_k;

    bandpass->y_k_2 = bandpass->y_k_1;

    bandpass->y_k_1 = bandpass->y_k;

}


void NotchFilterInit(Notch_Filter_Info_t* restrict notchFilterInfo, float a1, float a2, float b0, float b1, float b2) {

    // These values are hardcoded based on calculation in
    // MATLAB script "ResonantFilter_NotchFilter.m"
    // The convention which was used is the following. Since the MATLAB function iirpeak()
    // returns coefficients [b, a] in the 1-index notation and the C code is based on
    // the 0-index notation the coefficient are assigned accordingly.
    // Final remark is on the sign convention used in the following implementation.
    // The coefficients in a[2] and a[3] have to have changed the sign and the filter
    // parameters become -a[2], -a[3],

    notchFilterInfo->a1 =  a1;  // a1 = -a[2]
    notchFilterInfo->a2 =  a2;  // a2 = -a[3]

    notchFilterInfo->b0 =  b0;  // b0 =  b[1]
    notchFilterInfo->b1 =  b1;  // b1 =  b[2]
    notchFilterInfo->b2 =  b2;  // b2 =  b[3]

    // Initialize a filter variables
    notchFilterInfo->in    = 0;
    notchFilterInfo->in_1  = 0;
    notchFilterInfo->in_2  = 0;
    notchFilterInfo->out   = 0;
    notchFilterInfo->out_1 = 0;
    notchFilterInfo->out_2 = 0;
}

void NotchFilter(Notch_Filter_Info_t* restrict notchFilterInfo) {

    // MATLAB code from "ResonantFilter_NotchFilter.m"
    // y(k) = a1 * y(k - 1) + a2 * y(k - 2) + b0 * u(k) + b1 * u(k - 1) + b2 * u(k - 2);

    notchFilterInfo->out = notchFilterInfo->a1 * notchFilterInfo->out_1 + notchFilterInfo->a2 * notchFilterInfo->out_2 +
                           notchFilterInfo->b0 * notchFilterInfo->in    + notchFilterInfo->b1 * notchFilterInfo->in_1  + notchFilterInfo->b2 * notchFilterInfo->in_2;

    notchFilterInfo->in_2  = notchFilterInfo->in_1;
    notchFilterInfo->in_1  = notchFilterInfo->in;

    notchFilterInfo->out_2 = notchFilterInfo->out_1;
    notchFilterInfo->out_1 = notchFilterInfo->out;

}

void LpInit(lp_filter_t* lp_filter, float fo, float fs) {

    float temp = 0;

    temp = (2.0 * ((float) TWO_PI) / ((float) fs)) * ((float) fo);

    lp_filter->param = (2.0 * sin(temp)) * sin(temp);

    lp_filter->in = 0;

    lp_filter->out_k = lp_filter->out_k_1 = 0;

}

void LpFilter(lp_filter_t* lp_filter) {

    lp_filter->out_k = lp_filter->out_k_1 + lp_filter->param * (lp_filter->in - lp_filter->out_k_1);

    lp_filter->out_k_1 = lp_filter->out_k;


}

void SimpleIntgInit(sig_t* sig, float T) {

    sig->T = T;
    sig->out_k = sig->out_k_1 = 0;
    sig->in_k = 0;
}

void SimpleIntg(sig_t* sig) {

    sig->out_k = sig->out_k_1 + sig->T * sig->in_k;

    sig->out_k_1 = sig->out_k;
}

//void SmaFilterInit(Sma_Filter_Info_t* restrict smaFilterInfo, float Nsamples) {
//
//    smaFilterInfo->in = 0;
//
//    smaFilterInfo->out = 0;
//    smaFilterInfo->out_1 = 0;
//
//    smaFilterInfo->arate = (Nsamples - 1.0) / Nsamples;
//    smaFilterInfo->brate = 1.0 / Nsamples;
//
//}
//
//void SmaFilter(Sma_Filter_Info_t* restrict smaFilterInfo) {
//
//    smaFilterInfo->out = smaFilterInfo->arate * smaFilterInfo->out_1 + smaFilterInfo->brate * smaFilterInfo->in;
//
//    smaFilterInfo->out_1 = smaFilterInfo->out;
//
//}
