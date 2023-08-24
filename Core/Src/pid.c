/*
 * pid.c
 *
 *  Created on: Aug 12, 2023
 *      Author: johnson
 */

#include "pid.h"
#include "pd_aux_typedef.h"
PID_struct_t Vout1_pid_param;
PID_struct_t Vout2_pid_param;
/*
 * Initialize PID parameter for two output channel
 * Vout1 is initialize to 3v3 output parameter
 * Vout2 is initialize to 12v output parameter
 * If output voltage changed by switch, the parameter need to be updated for
 * good transient response
 */
void Init_pid(PID_struct_t* pid)
{
	if(pid == &Vout1_pid_param){
	    // Set PID Coefficients
		pid->Kp = 0.2;
		pid->Ki = 0.001;
		pid->Kd = 0;

	    // Set PID Setpoint
		pid->target_value = _3v3_ADC_target;
		pid->output_max = PWM1_MAX_DUTY;
		pid->output_min = PWM1_MIN_DUTY;
		pid->err_1 = 0.0f;
		pid->err_2 = 0.0f;
	}
	else if (pid == &Vout2_pid_param){
		// Set PID Coefficients
		pid->Kp = 0.3;
		pid->Ki = 0.001;
		pid->Kd = 0;

		// Set PID Setpoint
		pid->target_value = _12v_ADC_target;
		pid->output_max = PWM2_MAX_DUTY;
		pid->output_min = PWM2_MIN_DUTY;
		pid->err_1 = 0.0f;
		pid->err_2 = 0.0f;
	}
	else
		return;
}


void pid_process(PID_struct_t* pid)
{
    // Compute the error
    pid->err = -(pid->target_value - pid->adc_fb_value);

    // Compute the proportional output
    pid->inc_output = pid->Kp * (pid->err - pid->err_1);

    // Compute the integral output
    pid->inc_output += pid->Ki * (pid->err + pid->err_1);

    // Compute the derivative output
    pid->inc_output += pid->Kd * (pid->err - 2 * pid->err_1 + pid->err_2);

    pid->output = pid->last_output + pid->inc_output;

    // Saturate the output
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    // Update the previous output
    pid->last_output = pid->output;

    // Update last error in time domain
    pid->err_2 = pid->err_1;
    pid->err_1 = pid->err;
}
