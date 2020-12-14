/*
 * PQ.h
 *
 *  Created on: 17 Oct. 2020
 *      Author: Worker
 */

#ifndef HEADERS_PQ_H_
#define HEADERS_PQ_H_


typedef struct SYNCHRO_3F_INFO {

        float Ts;
        float Fo;

        float G;
        float D;

        float synchro_eA_t;
        float synchro_eA_t_1;

        float synchro_eB_t;
        float synchro_eB_t_1;

        float synchro_e_t_1;

        float synchro_Wo_t;
        float synchro_Wo_t_1;

        float synchro_wo_t;
        float synchro_wo_t_1;

        float uA_t;
        float uA_t_1;

        float uB_t;
        float uB_t_1;

        float ef_t;
        float ef_t_1;

        float kg_t;
        float kg_t_1;

        float sumA_at_t;
        float sumA_at_t_1;

        float sumB_at_t;
        float sumB_at_t_1;

        float xa_pos;
        float xb_pos;

        float xa_neg;
        float xb_neg;

        float ampl_sqrt;

} synchro_3f_info_t;

typedef struct SAMPLE_INFO {

    float uA_t;
    float uB_t;

} sample_info_t;

void synchro_3f_init(synchro_3f_info_t *synchro_3f_info, float Ts, float Fo, float G, float D, float K);
void fll_3f(sample_info_t *sample_info, synchro_3f_info_t *synchro_3f_info);


#endif /* HEADERS_PQ_H_ */
