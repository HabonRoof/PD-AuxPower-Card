/*
 * pd_aux_typedef.h
 *
 *  Created on: Jul 28, 2023
 *      Author: johnson
 */

#ifndef INC_PD_AUX_TYPEDEF_H_
#define INC_PD_AUX_TYPEDEF_H_

//*********************************************
// Macros for LEDs
//*********************************************
#define _24vLED_ON		HAL_GPIO_WritePin(LED_24v_GPIO_Port, LED_24v_Pin, GPIO_PIN_RESET)
#define _24vLED_OFF		HAL_GPIO_WritePin(LED_24v_GPIO_Port, LED_24v_Pin, GPIO_PIN_SET)
#define _24vLED_TOGGLE	HAL_GPIO_TogglePin(LED_24v_GPIO_Port, LED_24v_Pin)
#define _12vLED_ON		HAL_GPIO_WritePin(LED_12v_GPIO_Port, LED_12v_Pin, GPIO_PIN_RESET)
#define _12vLED_OFF		HAL_GPIO_WritePin(LED_12v_GPIO_Port, LED_12v_Pin, GPIO_PIN_SET)
#define _12vLED_TOGGLE	HAL_GPIO_TogglePin(LED_12v_GPIO_Port, LED_12v_Pin)
#define _5vLED_ON		HAL_GPIO_WritePin(LED_5v_GPIO_Port, LED_5v_Pin, GPIO_PIN_RESET)
#define _5vLED_OFF		HAL_GPIO_WritePin(LED_5v_GPIO_Port, LED_5v_Pin, GPIO_PIN_SET)
#define _5vLED_TOGGLE	HAL_GPIO_TogglePin(LED_5v_GPIO_Port, LED_5v_Pin)
#define _3v3LED_ON		HAL_GPIO_WritePin(LED_3v3_GPIO_Port, LED_3v3_Pin, GPIO_PIN_RESET)
#define _3v3LED_OFF		HAL_GPIO_WritePin(LED_3v3_GPIO_Port, LED_3v3_Pin, GPIO_PIN_SET)
#define _3v3LED_TOGGLE	HAL_GPIO_TogglePin(LED_3v3_GPIO_Port, LED_3v3_Pin)

//*********************************************
// Output target value
//*********************************************
#define _3v3_ADC_target	4040
#define _5v_ADC_target 	3913
#define _12v_ADC_target	3995
#define _24v_ADC_target	3787

//*********************************************
// PWM Limits
//*********************************************
// Duty limit of 3.3/5v rail
#define PWM1_MAX_DUTY	100		// 159 = 500kHz
#define PWM1_MIN_DUTY	0

// Duty limit of 12/24v rail
#define PWM2_MAX_DUTY	100		// 159 = 500kHz
#define PWM2_MIN_DUTY	0

typedef enum {
	BOARD_OK, PD_ERROR, TIMER_ERROR, ADC_ERROR,
	I2C_ERROR, UART_ERROR, BOARD_ERROR
} Board_status_t;

typedef enum {
	SMPS_OK, SMPS_ERROR, Vin_OVP, _3V3_OVP,
	_5V_OVP, _12V_OVP, _24V_OVP, OCP
} Power_status_t;

#endif /* INC_PD_AUX_TYPEDEF_H_ */
