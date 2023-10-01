/*
 * FUSB302B_driver.c
 *
 *  Created on: Aug 19, 2023
 *      Author: johnson
 */

#include "FUSB302B_driver.h"

uint8_t pd_buffer[PD_BUF_LENGTH] = {0};
uint8_t	pd_message[PD_MESSAGE_LENGTH] = {0};

FUSB302B_t fusb = {
		.addr 		= FUSB302B_ADDR,
		.cc_pin		= CC_NONE,
		.reg		= Reg_DeviceID,
		.buffer		= pd_buffer,
		.status		= FUSB302B_OK,
		.rx_sts		= RXFIFO_EMPTY,
		.tx_sts		= TXFIFO_EMPTY,
};

PD_protocol_t pd = {
		.cc			= CC_FAILED,
		.message	= pd_message,
};

/* @brief check is FUSB302B valid
 * @param	*hi2c				specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_probe(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){

	HAL_StatusTypeDef hal_status;
	// check response at FUSB302B_ADDR for 3 times, timeout 1000mS
	hal_status = HAL_I2C_IsDeviceReady( hi2c, FUSB302B_ADDR, 3, 1000);

	if (hal_status != HAL_OK)
		pfusb->status = FUSB302B_ERROR;
	else
		pfusb->status = FUSB302B_OK;
	return pfusb->status;
}

/* @brief read all FUSB302B register
 * @param	*hi2c				specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_read_all(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){

	HAL_StatusTypeDef hal_status;
	// check response at FUSB302B_ADDR for 3 times, timeout 1000mS
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_DeviceID, 1);
	pfusb->reg.deviceID = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Switches0, 1);
	pfusb->reg.switches0 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Switches1, 1);
	pfusb->reg.switches1 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Measure, 1);
	pfusb->reg.measure = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Slice, 1);
	pfusb->reg.slice = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Control0, 1);
	pfusb->reg.control0 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Control1, 1);
	pfusb->reg.control1 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Control2, 1);
	pfusb->reg.control2 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Control3, 1);
	pfusb->reg.control3 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Mask1, 1);
	pfusb->reg.mask1 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Power, 1);
	pfusb->reg.power = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_OCPreg, 1);
	pfusb->reg.OCPreg = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Maska, 1);
	pfusb->reg.maska = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Maskb, 1);
	pfusb->reg.maskb = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Control4, 1);
	pfusb->reg.control4 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Status0a, 1);
	pfusb->reg.status0a = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Status1a, 1);
	pfusb->reg.status1a = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Interrupta, 1);
	pfusb->reg.interrupta = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Interruptb, 1);
	pfusb->reg.interruptb = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Status0, 1);
	pfusb->reg.status0 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Status1, 1);
	pfusb->reg.status1 = pfusb->buffer[1];
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Interrupt, 1);
	pfusb->reg.interrupt = pfusb->buffer[1];

	if (hal_status != HAL_OK)
		pfusb->status = FUSB302B_ERROR;
	else
		pfusb->status = FUSB302B_OK;
	return pfusb->status;
}

/* @brief 	initial FUSB302B chip
 * @param 	*hi2c				specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_init(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){
	HAL_StatusTypeDef hal_status;
	FUSB302B_status_t status;
	// check is FUSB302B connected
	hal_status = FUSB302B_probe(hi2c, pfusb);
	if(pfusb->status != FUSB302B_OK)
		return FUSB302B_ERROR;
	// reset FUSB302B
	status = FUSB302B_write_reg(hi2c, pfusb, Reg_Reset, FUSB302B_Reset_SW_RES);

	status &= FUSB302B_read_all(hi2c, pfusb);

	// power on FUSB302B internal module
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Power, pfusb->reg.power | FUSB302B_Power_PWR_InternalOscillator \
			| FUSB302B_Power_PWR_MeasurementBlock | FUSB302B_Power_PWR_RxAndCur4MB | FUSB302B_Power_PWR_BandgapAndWake);

	// set auto CRC ack
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Switches1, pfusb->reg.switches1 | FUSB302B_Switches1_POWERROLE_Sink \
			| FUSB302B_Switches1_DATAROLE_Sink | FUSB302B_Switches1_SPECREV_Rev2_0 | FUSB302B_Switches1_AUTO_CRC);

	// flush TX FIFO
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Control0, pfusb->reg.control0 | FUSB302B_Control0_TX_FLUSH);

	// flush RX FIFO
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Control1, pfusb->reg.control1 | FUSB302B_Control1_RX_FLUSH);

	// enable auto retry, auto soft reset, auto hard reset and set retry times
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Control3, pfusb->reg.control3 | FUSB302B_Control3_AUTO_HARDRESET | FUSB302B_Control3_AUTO_SOFTRESET \
			| FUSB302B_Control3_N_RETRIES_MASK |  FUSB302B_Control3_AUTO_RETRY );

	if (hal_status != HAL_OK)
		pfusb->status = FUSB302B_ERROR;
	else
		pfusb->status = FUSB302B_OK;
	return pfusb->status;
}

/* @brief	check which CC pin is connect to cable
 * @param	*hi2c			specific i2c HAL driver
 * @return	FUSB302B_status_t	FUSB302 status
 *
 */
FUSB302B_status_t FUSB302B_check_CC_pin(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){
	uint8_t BC_LVL1, BC_LVL2;
	HAL_StatusTypeDef hal_status;

	// If already establish CC connection
	if(pfusb->cc_pin != CC_NONE)
		return FUSB302B_OK;
	// check CC pin connection
	else{
		// connect DAC to CC1
		hal_status = FUSB302B_write_reg(hi2c, pfusb, Reg_Switches0, pfusb->reg.switches0 | FUSB302B_Switches0_MEAS_CC1 \
				| FUSB302B_Switches0_PDWN_MASK);
		HAL_Delay(250);
		hal_status &= FUSB302B_read_reg(hi2c, pfusb, Reg_Status0, 1);
		BC_LVL1 = pfusb->buffer[1] & FUSB302B_Status0_BC_LVL_MASK;

		// else set switch to connect CC2 pin
		hal_status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Switches0, pfusb->reg.switches0 | FUSB302B_Switches0_MEAS_CC2 \
				| FUSB302B_Switches0_PDWN_MASK);
		HAL_Delay(250);
		hal_status &= FUSB302B_read_reg(hi2c, pfusb, Reg_Status0, 1);
		BC_LVL2 = pfusb->buffer[1] & FUSB302B_Status0_BC_LVL_MASK;

		// return failed
		if (hal_status != HAL_OK)
			return FUSB302B_ERROR;

		if(BC_LVL1 > BC_LVL2)
			pfusb->cc_pin = CC_PIN1;
		else if(BC_LVL2 > BC_LVL1)
			pfusb->cc_pin = CC_PIN2;
		else
			pfusb->cc_pin = CC_NONE;
		return FUSB302B_OK;
	}
}

/* @brief	read RXFIFO and TXFIFO status
 * @ param	I2C_HandleTypeDef	*hi2c	specific i2c HAL driver
 * @ param	FUSB302B_t			*pfusb	fusb object pointer
 * @return	FUSB302B_status_t	status 	FUSB302 status
 */
FUSB302B_status_t FUSB302B_check_FIFO_status(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){
	HAL_StatusTypeDef hal_status;
	hal_status = FUSB302B_read_reg(hi2c, pfusb, Reg_Status1, 1);
	pfusb->rx_sts = (pfusb->buffer[1] & FUSB302B_Status1_RX_STATUS_MASK) >> 4;
	pfusb->tx_sts = (pfusb->buffer[1] & FUSB302B_Status1_TX_STATUS_MASK) >> 2;
	if (hal_status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief	start PD communication on CC pin
 * @ param	I2C_HandleTypeDef	*hi2c	specific i2c HAL driver
 * @ param	FUSB302B_t			*pfusb	fusb object pointer
 * @return	FUSB302B_status_t	status 	FUSB302 status
 */
FUSB302B_status_t FUSB302B_start_CC_communication(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){
	FUSB302B_status_t status;
	// update register data
	FUSB302B_read_all(hi2c, pfusb);

	switch(pfusb->cc_pin){
	case CC_NONE:
		return FUSB302B_OK;
		break;

	case CC_PIN1:
		// connect CC1 pin to DAC
		status = FUSB302B_write_reg(hi2c, pfusb, Reg_Switches0, (pfusb->reg.switches0 & ~FUSB302B_Switches0_MEAS_CC_MASK) \
				| FUSB302B_Switches0_MEAS_CC1);
		// connect CC1 to BMC decoder
		status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Switches1, (pfusb->reg.switches1 & ~FUSB302B_Switches1_TXCC_MASK) | FUSB302B_Switches1_TXCC1);
		break;

	case CC_PIN2:
		// connect CC1 pin to DAC
		status = FUSB302B_write_reg(hi2c, pfusb, Reg_Switches0, (pfusb->reg.switches0 & ~FUSB302B_Switches0_MEAS_CC_MASK) \
				| FUSB302B_Switches0_MEAS_CC2 );
		// connect CC1 to BMC decoder
		status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Switches1, (pfusb->reg.switches1 & ~FUSB302B_Switches1_TXCC_MASK) | FUSB302B_Switches1_TXCC2);
		break;
	default:
		break;
	}
	// reset PD logic
	status &= FUSB302B_write_reg(hi2c, pfusb, Reg_Reset, (pfusb->reg.reset & ~FUSB302B_Reset_Reserved_MASK) | FUSB302B_Reset_PD_RESET);
	FUSB302B_read_all(hi2c, pfusb);
	if (status != FUSB302B_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief	write data to register of FUSB302B
 * @param 	I2C_HandleTypeDef	*hi2c	specific i2c HAL driver
 * @param	FUSB320B_Reg_t		reg		register address of FUSB302
 * @param	uint8_t				*data	data to write to register
 * @return 	FUSB302B_status_t	status	FUSB302 status
 */
FUSB302B_status_t FUSB302B_write_reg(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb, FUSB320B_Reg_addr_t reg, uint8_t data){
	// to write to FUSB302, first transmit register address, then data sequence
	// assign register into pd_buffer
	pfusb->buffer[0] = (uint8_t)reg;
	pfusb->buffer[1] = data;
	HAL_StatusTypeDef hal_status;
	// transmit data through I2C
	hal_status = HAL_I2C_Master_Transmit(hi2c, fusb.addr, pfusb->buffer, 2, 500);

	if (hal_status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/* @brief	read data from register of FUSB302B
 * @param 	I2C_HandleTypeDef	*hi2c	specific i2c HAL driver
 * @param	FUSB320B_Reg_t		reg		register address of FUSB302
 * @param	uint8_t				*data	data to write to register
 * @param	uint16_t			len		length of data to read
 * @return 	FUSB302B_status_t	status	FUSB302 status
 */
FUSB302B_status_t FUSB302B_read_reg(I2C_HandleTypeDef* hi2c,FUSB302B_t* pfusb, FUSB320B_Reg_addr_t reg , uint16_t len){
	HAL_StatusTypeDef hal_status;
	pfusb->buffer[0] = (uint8_t)reg;

	// transmit read register request
	hal_status = HAL_I2C_Master_Transmit(hi2c, pfusb->addr, pfusb->buffer, 1, 500);

	if(hal_status == HAL_OK)
		hal_status &= HAL_I2C_Master_Receive(hi2c, pfusb->addr, pfusb->buffer + 1, len, 500);

	if (hal_status != HAL_OK)
		return FUSB302B_ERROR;
	else
		return FUSB302B_OK;
}

/*@brief	reset FUSB302 to initial state
 * @param 	*hi2c			specific i2c HAL driver
 * @return 	FUSB302B_status_t	FUSB302 status
 */
FUSB302B_status_t FUSB302B_deinit(I2C_HandleTypeDef* hi2c, FUSB302B_t* pfusb){
	// reset pd_buffer
	// memset(pfusb->buffer, 0, sizeof(pfusb->buffer));

	// reset FUSB302B
	HAL_StatusTypeDef status = FUSB302B_write_reg(hi2c, pfusb, Reg_Reset, FUSB302B_Reset_SW_RES);

	if(status != HAL_OK)
		return FUSB302B_ERROR;
	return FUSB302B_OK;
}



