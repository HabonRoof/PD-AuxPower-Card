/*
 * FUSB302B_driver.h
 *
 *  Created on: Aug 19, 2023
 *      Author: johnson
 */

#ifndef INC_FUSB302B_DRIVER_H_
#define INC_FUSB302B_DRIVER_H_
#include "stdint.h"
#include "stm32l4xx_hal.h"
#include "string.h"

//*********************************************
// Register address of FUSB302B PD controller
//*********************************************
#define FUSB302B_DeviceID_Version_MASK			0xF0
#define FUSB302B_DeviceID_Revision_MASK			0x0F

#define FUSB302B_Switches0_PU_EN_MASK			0b11000000
#define FUSB302B_Switches0_PU_EN2				0b10000000
#define FUSB302B_Switches0_PU_EN1				0b01000000
#define FUSB302B_Switches0_VCONN_CC_MASK		0b00110000
#define FUSB302B_Switches0_VCONN_CC2			0b00100000
#define FUSB302B_Switches0_VCONN_CC1			0b00010000
#define FUSB302B_Switches0_MEAS_CC_MASK			0b00001100
#define FUSB302B_Switches0_MEAS_CC2				b00001000
#define FUSB302B_Switches0_MEAS_CC1				0b00000100
#define FUSB302B_Switches0_PDWN_MASK			0b00000011
#define FUSB302B_Switches0_PDWN2				0b00000010
#define FUSB302B_Switches0_PDWN1				0b00000001

#define FUSB302B_Switches1_POWERROLE			0b10000000
#define FUSB302B_Switches1_SPECREV_MASK			0b01100000
#define FUSB302B_Switches1_DATAROLE				0b00010000
#define FUSB302B_Switches1_Reserved				0b00001000
#define FUSB302B_Switches1_AUTO_CRC				0b00000100
#define FUSB302B_Switches1_TXCC_MASK			0b00000011

#define FUSB302B_Switches1_SPECREV_Rev1_0		0b00000000
#define FUSB302B_Switches1_SPECREV_Rev2_0		0b00100000
#define FUSB302B_Switches1_POWERROLE_Source		0b10000000
#define FUSB302B_Switches1_POWERROLE_Sink		0b00000000
#define FUSB302B_Switches1_DATAROLE_Source		0b00010000
#define FUSB302B_Switches1_DATAROLE_Sink		0b00000000
#define FUSB302B_Switches1_TXCC2				0b00000010
#define FUSB302B_Switches1_TXCC1				0b00000001

#define FUSB302B_Measure_Reserved				0b10000000
#define FUSB302B_Measure_MEAS_VBUS				0b01000000
#define FUSB302B_Measure_MDAC_MASK				0b00111111

#define FUSB302B_Slice_SDAC_HYS_MASK			0b11000000
#define FUSB302B_Slice_SDAC_MASK				0b00111111

#define FUSB302B_Control0_Reserved_MASK			0b10010000
#define FUSB302B_Control0_TX_FLUSH				0b01000000
#define FUSB302B_Control0_INT_MASK				0b00100000
#define FUSB302B_Control0_HOST_CUR_MASK			0b00001100
#define FUSB302B_Control0_AUTO_PRE				0b00000010
#define FUSB302B_Control0_TX_START				0b00000001

#define FUSB302B_Control0_HOST_CUR_NoCurrent			0b00000000
#define FUSB302B_Control0_HOST_CUR_DefaultUSBPower		0b00000100
#define FUSB302B_Control0_HOST_CUR_MediumCurrentMode	0b00001000
#define FUSB302B_Control0_HOST_CUR_HighCurrentMode		0b00001100

#define FUSB302B_Control1_Reserved_MASK			0b10001000
#define FUSB302B_Control1_ENSOP2DB				0b01000000
#define FUSB302B_Control1_ENSOP1DB				0b00100000
#define FUSB302B_Control1_BIST_MODE2			0b00010000
#define FUSB302B_Control1_RX_FLUSH				0b00000100
#define FUSB302B_Control1_ENSOP2				0b00000010
#define FUSB302B_Control1_ENSOP1				0b00000001

#define FUSB302B_Control2_Reserved_MASK			0b00010000
#define FUSB302B_Control2_TOG_SAVE_MASK			0b11000000
#define FUSB302B_Control2_TOG_DR_ONLY			0b00100000
#define FUSB302B_Control2_WAKE_EN				0b00001000
#define FUSB302B_Control2_MODE_MASK				0b00000110
#define FUSB302B_Control2_TOGGLE				0b00000001

#define FUSB302B_Control2_MODE_SrcPolling		0b00000110
#define FUSB302B_Control2_MODE_SnkPolling		0b00000100
#define FUSB302B_Control2_MODE_DrpPolling		0b00000010
#define FUSB302B_Control2_MODE_Off				0b00000000

#define FUSB302B_Control3_Reserved_MASK			0b10100000
#define FUSB302B_Control3_SEND_HARD_RESET		0b01000000
#define FUSB302B_Control3_AUTO_HARDRESET		0b00010000
#define FUSB302B_Control3_AUTO_SOFTRESET		0b00001000
#define FUSB302B_Control3_N_RETRIES_MASK		0b00000110
#define FUSB302B_Control3_AUTO_RETRY			0b00000001

#define FUSB302B_Mask1_M_VBUSOK					0b10000000
#define FUSB302B_Mask1_M_ACTIVITY				0b01000000
#define FUSB302B_Mask1_M_COMP_CHNG				0b00100000
#define FUSB302B_Mask1_M_CRC_CHK				0b00010000
#define FUSB302B_Mask1_M_ALERT					0b00001000
#define FUSB302B_Mask1_M_WAKE					0b00000100
#define FUSB302B_Mask1_M_COLLISION				0b00000010
#define FUSB302B_Mask1_M_BC_LVL					0b00000001

#define FUSB302B_Mask1_ALL						0xFF

#define FUSB302B_Power_Reserved_MASK			0b11110000
#define FUSB302B_Power_PWR_MASK					0b00001111

#define FUSB302B_Power_PWR_InternalOscillator 	0b00001000
#define FUSB302B_Power_PWR_MeasurementBlock		0b00000100
#define FUSB302B_Power_PWR_RxAndCur4MB			0b00000010
#define FUSB302B_Power_PWR_BandgapAndWake		0b00000001

#define FUSB302B_Reset_Reserved_MASK			0b11111100
#define FUSB302B_Reset_PD_RESET					0b00000010
#define FUSB302B_Reset_SW_RES					0b00000001

#define FUSB302B_OCPreg_Reserved_MASK			0b11110000
#define FUSB302B_OCPreg_OCP_RANGE				0b00001000
#define FUSB302B_OCPreg_OCP_CUR_MASK			0b00000111

#define FUSB302B_Maska_M_OCP_TEMP				0b10000000
#define FUSB302B_Maska_M_TOGDONE				0b01000000
#define FUSB302B_Maska_M_SOFTFAIL				0b00100000
#define FUSB302B_Maska_M_RETRYFAIL				0b00010000
#define FUSB302B_Maska_M_HARDSENT				0b00001000
#define FUSB302B_Maska_M_TXSENT					0b00000100
#define FUSB302B_Maska_M_SOFTRST				0b00000010
#define FUSB302B_Maska_M_HARDRST				0b00000001

#define FUSB302B_Maska_ALL						0xFF

#define FUSB302B_Maskb_Reserved_MASK			0b11111110
#define FUSB302B_Maskb_M_GCRCSENT				0b00000001

#define FUSB302B_Maskb_ALL						0b00000001

#define FUSB302B_Control4_Reserved_MASK			0b11111110
#define FUSB302B_Control4_TOG_USRC_EXIT			0b00000001

#define FUSB302B_Status0a_Reserved_MASK			0b11000000
#define FUSB302B_Status0a_SOFTFAIL				0b00100000
#define FUSB302B_Status0a_RETRYFAIL				0b00010000
#define FUSB302B_Status0a_POWER_MASK			0b00001100
#define FUSB302B_Status0a_SOFTRST				0b00000010
#define FUSB302B_Status0a_HARDRST				0b00000001

#define FUSB302B_Status1a_Reserved_MASK			0b11000000
#define FUSB302B_Status1a_TOGSS					0b00111000
#define FUSB302B_Status1a_RXSOP2DB				0b00000100
#define FUSB302B_Status1a_RXSOP1DB				0b00000010
#define FUSB302B_Status1a_RXSOP					0b00000001

#define FUSB302B_Status1a_TOGSS_Running			0b00000000
#define FUSB302B_Status1a_TOGSS_SRC_on_CC1		0b00001000
#define FUSB302B_Status1a_TOGSS_SRC_on_CC2		0b00010000
#define FUSB302B_Status1a_TOGSS_SNK_on_CC1		0b00101000
#define FUSB302B_Status1a_TOGSS_SNK_on_CC2		0b00110000
#define FUSB302B_Status1a_TOGSS_AudioAccessory 	0b00111000

#define FUSB302B_Interrupta_I_OCP_TEMP			0b10000000
#define FUSB302B_Interrupta_I_TOGDONE			0b01000000
#define FUSB302B_Interrupta_I_SOFTFAIL			0b00100000
#define FUSB302B_Interrupta_I_RETRYFAIL			0b00010000
#define FUSB302B_Interrupta_I_HARDSENT			0b00001000
#define FUSB302B_Interrupta_I_TXSENT			0b00000100
#define FUSB302B_Interrupta_I_SOFTRST			0b00000010
#define FUSB302B_Interrupta_I_HARDRST			0b00000001

#define FUSB302B_Interrupta_I_ALL				0xFF

#define FUSB302B_Interruptb_Reserved_MASK		0b11111110
#define FUSB302B_Interruptb_I_GCRCSENT			0b00000001

#define FUSB302B_Interruptb_I_ALL				0b00000001

#define FUSB302B_Status0_VBUSOK					0b10000000
#define FUSB302B_Status0_ACTIVITY				0b01000000
#define FUSB302B_Status0_COMP					0b00100000
#define FUSB302B_Status0_CRC_CHK				0b00010000
#define FUSB302B_Status0_ALERT					0b00001000
#define FUSB302B_Status0_WAKE					0b00000100
#define FUSB302B_Status0_BC_LVL_MASK			0b00000011

#define FUSB302B_Status0_BC_LVL_LessThan200mV  	0b00000000
#define FUSB302B_Status0_BC_LVL_200mV_to_660mV 	0b00000001
#define FUSB302B_Status0_BC_LVL_660mV_to_1230mV 0b00000010
#define FUSB302B_Status0_BC_LVL_MoreThan1230mV 	0b00000011

#define FUSB302B_Status1_RXSOP2				0b10000000
#define FUSB302B_Status1_RXSOP1				0b01000000
#define FUSB302B_Status1_RX_EMPTY			0b00100000
#define FUSB302B_Status1_RX_FULL			0b00010000
#define FUSB302B_Status1_TX_EMPTY			0b00001000
#define FUSB302B_Status1_TX_FULL			0b00000100
#define FUSB302B_Status1_OVRTEMP			0b00000010
#define FUSB302B_Status1_OCP				0b00000001

#define FUSB302B_Interrupt_I_VBUSOK			0b10000000
#define FUSB302B_Interrupt_I_ACTIVITY		0b01000000
#define FUSB302B_Interrupt_I_COMP_CHNG		0b00100000
#define FUSB302B_Interrupt_I_CRC_CHK		0b00010000
#define FUSB302B_Interrupt_I_ALERT			0b00001000
#define FUSB302B_Interrupt_I_WAKE			0b00000100
#define FUSB302B_Interrupt_I_COLLISION		0b00000010
#define FUSB302B_Interrupt_I_BC_LVL			0b00000001

#define FUSB302B_Interrupt_I_ALL		0xFF

#define FUSB302B_FIFOs_TXRX_Token_MASK	0xFF

#define	FUSB302B_TxFIFOToken_TXON		0xA1
#define	FUSB302B_TxFIFOToken_SOP1		0x12
#define	FUSB302B_TxFIFOToken_SOP2		0x13
#define	FUSB302B_TxFIFOToken_SOP3		0x1B
#define	FUSB302B_TxFIFOToken_RESET1		0x15
#define	FUSB302B_TxFIFOToken_RESET2		x16
#define	FUSB302B_TxFIFOToken_PACKSYM	0x80
#define	FUSB302B_TxFIFOToken_JAM_CRC	0xFF
#define	FUSB302B_TxFIFOToken_EOP		0x14
#define	FUSB302B_TxFIFOToken_TXOFF		0xFE

#define FUSB302B_RxFIFOToken_MASK			0b11100000
#define FUSB302B_RxFIFOToken_OFFSET			5

#define FUSB302B_RxFIFOToken_SOP			0b11100000
#define FUSB302B_RxFIFOToken_SOP1			0b11000000
#define FUSB302B_RxFIFOToken_SOP2			0b10100000
#define FUSB302B_RxFIFOToken_SOP1DB			0b10000000
#define FUSB302B_RxFIFOToken_SOP2DB			0b01100000

#define PD_BUF_LENGTH			8
#define FUSB302B_ADDR			0x22 << 1

typedef enum{
	Reg_DeviceID 	= 0x01,
	Reg_Switches0 	= 0x02,
	Reg_Switches1 	= 0x03,
	Reg_Measure 	= 0x04,
	Reg_Slice 		= 0x05,
	Reg_Control0 	= 0x06,
	Reg_Control1 	= 0x07,
	Reg_Control2 	= 0x08,
	Reg_Control3 	= 0x09,
	Reg_Mask1	 	= 0x0A,
	Reg_Power	 	= 0x0B,
	Reg_Reset	 	= 0x0C,
	Reg_OCPreg	 	= 0x0D,
	Reg_Maska	 	= 0x0E,
	Reg_Maskb	 	= 0x0F,
	Reg_Control4	= 0x10,
	Reg_Status0a	= 0x3C,
	Reg_Status1a	= 0x3D,
	Reg_Interrupta	= 0x3E,
	Reg_Interruptb	= 0x3F,
	Reg_Status0		= 0x40,
	Reg_Status1		= 0x41,
	Reg_Interrupt 	= 0x42,
	Reg_FIFOs		= 0x43,
} FUSB320B_Reg_t;

typedef enum {
	FUSB302B_OK,
	FUSB302B_ERROR,
} FUSB302B_status_t;

typedef struct {
	uint16_t			addr;		// FUSB302B I2C 7-bit address
	FUSB320B_Reg_t		reg;
	uint8_t*			buff;		// PD protocol message buffer
	FUSB302B_status_t	status;
	FUSB302B_status_t	(*deinit)(I2C_HandleTypeDef* hi2c);		// reset FUSB302B
	FUSB302B_status_t	(*write_reg)(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg, uint8_t data);
	FUSB302B_status_t	(*read_reg)(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg ,uint8_t *data, uint16_t len);
} FUSB302B_t;

FUSB302B_status_t FUSB302B_deinit(I2C_HandleTypeDef* hi2c);
FUSB302B_status_t FUSB302B_write_reg(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg, uint8_t data);
FUSB302B_status_t FUSB302B_read_reg(I2C_HandleTypeDef* hi2c, FUSB320B_Reg_t reg ,uint8_t *data, uint16_t len);
FUSB302B_status_t FUSB302B_probe(I2C_HandleTypeDef* hi2c);
FUSB302B_status_t FUSB302B_init(I2C_HandleTypeDef* hi2c);

#endif /* INC_FUSB302B_DRIVER_H_ */
