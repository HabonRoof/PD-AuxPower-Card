/*
 * FUSB302B_driver.c
 *
 *  Created on: Aug 19, 2023
 *      Author: johnson
 */

#include "FUSB302B_driver.h"

uint8_t pd_buffer[PD_BUF_LENGTH] = {0};

/* @brief check is FUSB302B valid
 * @param	*hi2c				specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_probe(I2C_HandleTypeDef* hi2c){
	// check response at FUSB302B_ADDR for 3 times, timeout 100mS
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady( hi2c, FUSB302B_ADDR, 3, 1000);

	if (status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief 	initial FUSB302B chip
 * @param 	*hi2c				specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_init(I2C_HandleTypeDef* hi2c){

	// check is FUSB302B connected
	HAL_StatusTypeDef status = FUSB302B_probe(hi2c);
	if(status != HAL_OK)
		return FUSB302B_ERROR;

	// read FUSB302B device ID
	status &= FUSB302B_read_reg(hi2c, Reg_DeviceID, pd_buffer, 1);

	// reset FUSB302B
	status &= FUSB302B_write_reg(hi2c, Reg_Reset, FUSB302B_Reset_SW_RES);
	if(status != HAL_OK)
		return FUSB302B_ERROR;

	// flush TX FIFO
	status &= FUSB302B_write_reg(hi2c, Reg_Control0, FUSB302B_Control0_TX_FLUSH);
	status &= FUSB302B_read_reg(hi2c, Reg_Control0, pd_buffer, 1);
	if(status != HAL_OK)
		return FUSB302B_ERROR;

	// flush RX FIFO
	status &= FUSB302B_write_reg(hi2c, Reg_Control1, FUSB302B_Control1_RX_FLUSH);
	status &= FUSB302B_read_reg(hi2c, Reg_Control1, pd_buffer, 1);
	if(status != HAL_OK)
		return FUSB302B_ERROR;

	// set auto CRC ack
	status &= FUSB302B_write_reg(hi2c, Reg_Switches1, FUSB302B_Switches1_AUTO_CRC);
	status &= FUSB302B_read_reg(hi2c, Reg_Switches1, pd_buffer, 1);

	if(status != HAL_OK)
		return FUSB302B_ERROR;

	// enable auto retry, auto soft reset, auto hard reset and set retry times

	status &= FUSB302B_write_reg(hi2c, Reg_Control3,
			FUSB302B_Control3_AUTO_RETRY | (FUSB302B_Control3_N_RETRIES_MASK & 0x02 << 1) \
			| FUSB302B_Control3_AUTO_SOFTRESET 	| FUSB302B_Control3_AUTO_HARDRESET);
	status &= FUSB302B_read_reg(hi2c, Reg_Control3, pd_buffer, 1);

	if(status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief	write data to register of FUSB302B
 * @param 	*hi2c			specific i2c HAL driver
 * 			reg				register address of FUSB302
 * 			*data			data to write to register
 * 			len				data length to write
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_write_reg(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg, uint8_t data){
	// to write to FUSB302, first transmit register address, then data sequence
	// assign register into pd_buffer
	pd_buffer[0] = (uint8_t)reg;
	pd_buffer[1] = data;

	// transmit data through I2C
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, FUSB302B_ADDR, pd_buffer, 2, 500);

	if (status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief	read data from register of FUSB302B
 * @param 	*hi2c			specific i2c HAL driver
 * 			reg				register address of FUSB302
 * 			*data			data to read from register
 * 			len				data length to read
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_read_reg(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg ,uint8_t *data, uint16_t len){

	data[0] = (uint8_t)reg;
	// transmit read register request
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, FUSB302B_ADDR, data, 1, 500);

	if(status == HAL_OK){
		status &= HAL_I2C_Master_Receive(hi2c, FUSB302B_ADDR, (data + 1), len, 500);
	}
	if (status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/*@brief	reset FUSB302 to initial state
 * @param 	*hi2c			specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_deinit(I2C_HandleTypeDef* hi2c){
	// reset pd_buffer
	memset(pd_buffer, 0, sizeof(pd_buffer));

	// reset FUSB302B
	HAL_StatusTypeDef status = FUSB302B_write_reg(hi2c, Reg_Reset, FUSB302B_Reset_SW_RES);

	if(status != HAL_OK)
		return FUSB302B_ERROR;
	return FUSB302B_OK;
}

FUSB302B_t fusb = {
		.addr 		= FUSB302B_ADDR,
		.reg		= Reg_DeviceID,
		.buff		= pd_buffer,
		.status		= FUSB302B_OK,
		.deinit		= FUSB302B_deinit,
		.write_reg	= FUSB302B_write_reg,
		.read_reg	= FUSB302B_read_reg,
};

