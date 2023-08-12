/*
 * pid.h
 *
 *  Created on: Aug 12, 2023
 *      Author: johnson
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stdint.h"

typedef struct {
    uint16_t target_value;
    uint16_t adc_fb_value;

    float err;
    float err_1;
    float err_2;

    float Kp;
    float Ki;
    float Kd;

    float last_output;
    float inc_output;
    float output;
    uint16_t output_max;
    uint16_t output_min;
}PID_struct_t;


extern void Init_pid(PID_struct_t* pid);
extern void pid_process(PID_struct_t* pid);


#endif /* INC_PID_H_ */
