/*
 * adc.h
 *
 *  Created on: 22 Feb. 2020
 *      Author: Dejan Jovanovic
 */

#ifndef PROJECT_HEADERS_ADC_H_
#define PROJECT_HEADERS_ADC_H_



#define ADCCORE_V_COUNT_SCALE           ((3.0)/(4096.0))
#define ADCCORE_IDC_SCALE               (0.3333)
#define ADCCORE_VDC_SCALE               (0.008)


typedef struct  {

    float ia;
    float va;

    float ib;
    float vb;

    float ic;
    float vc;

    int inc;
    int N;


} ADC_Calibra_t;


typedef struct  {

    float ia;
    float va;

    float ib;
    float vb;

    float ic;
    float vc;

    float idc;
    float vdc;

    float vdrift;

} ADC_Read_t;

typedef struct  {

    float scale_ia;
    float offset_ia;

    float scale_va;
    float offset_va;

    float scale_ib;
    float offset_ib;

    float scale_vb;
    float offset_vb;

    float scale_ic;
    float offset_ic;

    float scale_vc;
    float offset_vc;

} ADC_Params_t;

typedef struct  {

    float ia;
    float va;

    float ib;
    float vb;

    float ic;
    float vc;

    float idc;
    float vdc;

    float vdrift;

} ADC_Read_Norm_t;

void Adc_Config(void);



#endif /* PROJECT_HEADERS_ADC_H_ */
