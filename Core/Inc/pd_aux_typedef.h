/*
 * pd_aux_typedef.h
 *
 *  Created on: Jul 28, 2023
 *      Author: johnson
 */

#ifndef INC_PD_AUX_TYPEDEF_H_
#define INC_PD_AUX_TYPEDEF_H_

#define FUSB302B_ADDR 	0x22 << 1
#define REG_DEVICE_ID	0x01
#define REG_SWITCH0		0x02
#define REG_SWITCHE1	0x03
#define REG_MEASURE		0x04
#define REG_SLICE		0x05
#define REG_CONTROL0	0x06
#define REG_CONTROL1	0x07
#define REG_CONTROL2	0x08
#define REG_CONTROL3	0x09
#define REG_MASK1		0x0A
#define REG_POWER		0x0B
#define REG_RESET		0x0C
#define REG_OCPreg		0x0D
#define REG_MASKA		0x0E
#define REG_MASKB		0x0F
#define REG_CONTROL4	0x10
#define REG_STATUS0A	0x3C
#define REG_STATUS1A	0x3D
#define REG_INTERRUPTA	0x3E
#define REG_INTERRUPTB	0x3F
#define REG_STATUS0		0x40
#define REG_STATUS1		0x41
#define REG_INTERRUPT	0x42
#define REG_FIFOS		0x43

typedef enum {
	BOARD_OK, PD_ERROR, TIMER_ERROR, ADC_ERROR,
	I2C_ERROR, UART_ERROR, BOARD_ERROR
} Board_status_t;

typedef enum {
	SMPS_OK, SMPS_ERROR, Vin_OVP, _3V3_OVP,
	_5V_OVP, _12V_OVP, _24V_OVP, OCP
} Power_status_t;

#endif /* INC_PD_AUX_TYPEDEF_H_ */