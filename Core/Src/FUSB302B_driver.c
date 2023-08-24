/*
 * FUSB302B_driver.c
 *
 *  Created on: Aug 19, 2023
 *      Author: johnson
 */

#include "FUSB302B_driver.h"


uint8_t pd_buffer[PD_MSG_DATA_LENGTH];

FUSB302B_status FUSB302B_write_reg(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t *data, uint16_t len){
	uint8_t buf[len + 1];

	// assign register into temp_buf
	buf[0] = reg;

	// copy message fron fusb.buffer to temp_buf
	memcpy((buf+1) , data, len);

	// transmit data through I2C
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, FUSB302B_ADDR, buf, len + 1, 1000);

	if (status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

FUSB302B_status FUSB302B_read_reg(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg ,uint8_t *data, uint16_t len){
	uint8_t buf[len];
	buf[0] = reg;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, FUSB302B_ADDR, buf, 1, 1000);
	if(status == HAL_OK){
		status &= HAL_I2C_Master_Receive(hi2c, FUSB302B_ADDR, buf, len, 1000);
	}
	if (status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

FUSB302B_status FUSB302B_reset(I2C_HandleTypeDef* hi2c){
	return FUSB302B_OK;
}

FUSB302B_t fusb = {
		.addr 		= FUSB302B_ADDR,
		.reg		= Reg_DeviceID,
		.buff		= pd_buffer,
		.status		= FUSB302B_OK,
		.reset		= FUSB302B_reset,
		.write_reg	= FUSB302B_write_reg,
		.read_reg	= FUSB302B_read_reg,
};

