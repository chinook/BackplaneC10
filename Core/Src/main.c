/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "chinook_can_ids.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// I2C GPIO addresses
#define GPIO0_ADDR 0x42
#define GPIO1_ADDR 0x40


#define GPIO_GP0 0x00
#define GPIO_GP1 0x01
#define GPIO_OLAT_GP0 0x02
#define GPIO_OLAT_GP1 0x03
#define GPIO_POL_GP0 0x04
#define GPIO_POL_GP1 0x05
#define GPIO_DIR_GP0 0x06
#define GPIO_DIR_GP1 0x07
#define GPIO_INT_GP0 0x08
#define GPIO_INT_GP1 0x09

#define GPIO_NORMAL 0x00
#define GPIO_INVERTED 0x01

#define GPIO_OUTPUT 0x00
#define GPIO_INPUT 0x01

// INA226
#define INA226_BATT_ADDR 0x96
#define INA226_VOLANT_ADDR 0x90
#define INA226_C9_ADDR 0x80
// TODO: Changer pour valeurs soudees
#define INA226_C10_1_ADDR 0x82
#define INA226_C10_2_ADDR 0x84
#define INA226_C10_3_ADDR 0x86

#define INA226_CONFIG_REG 0x00
#define INA226_SHUNT_VOLTAGE_REG 0x01
#define INA226_BUS_VOLTAGE_REG 0x02
#define INA226_POWER_REG 0x03
#define INA226_CURRENT_REG 0x04
#define INA226_CALIB_REG 0x05

// Board defines
#define BOARD_C10_1 3
#define BOARD_C10_2 2
#define BOARD_C10_3 1
#define BOARD_C9	0

// Voltage defines
#define VOLTAGE_3V3 3
#define VOLTAGE_5V  2
#define VOLTAGE_24V 0

// INA indices
#define VOLANT_INA 5
#define BATT_INA 4
#define BOARD_C10_1_INA BOARD_C10_1
#define BOARD_C10_2_INA BOARD_C10_2
#define BOARD_C10_3_INA BOARD_C10_3
#define BOARD_C9_INA BOARD_C9

// INA226 constants
#define INA226_RSHUNT 0.002f        // 2m ohm
#define INA226_CURRENT_LSB 0.001f  // 1mA/bit
#define INA226_SHUNT_VOLTAGE_LSB 0.0000025f // 2.5 uV
#define INA226_BUS_VOLTAGE_LSB 0.00125f     // 1.25 mV
const float INA226_POWER_LSB = 25.0f * INA226_CURRENT_LSB;


#define NUM_INA226 6
static uint32_t INA226_ADDRs[6] = { INA226_C9_ADDR, INA226_C10_3_ADDR, INA226_C10_2_ADDR, INA226_C10_1_ADDR, INA226_BATT_ADDR, INA226_VOLANT_ADDR };


// LED defines
enum LEDS
{
	LED_ERROR = 0,	// D7
	LED_WARN,		// D8
	LED_CAN,		// D9
	LED4,			// D10
	LED3,			// D11
	LED2,			// D12
	LED1,			// D13

	NUM_LEDS
};

// State machine enum
enum STATES
{
	STATE_INIT = 0,
	STATE_ACQUISITION,
	STATE_BOARD_CONTROL,
	STATE_CAN,

	STATE_ERROR = 0xFF
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t timer_flag;
uint8_t b_acquisition_gpios_flag;
uint8_t b_flag_can_tx;

uint8_t pb1_pressed;
uint8_t pb2_pressed;

uint8_t can1_recv_flag;

CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailbox;

// I2C GPIO registers
uint8_t gpio0_gp0, gpio0_gp1; // LEDs + Pre-Prog + 24V status
uint8_t gpio1_gp0, gpio1_gp1; // Voltage enables
uint8_t gpio0_dir0, gpio0_dir1, gpio1_dir0, gpio1_dir1;
uint8_t gpio0_pol0, gpio0_pol1, gpio1_pol0, gpio1_pol1;

// I2C INA226 registers
uint16_t ina226_config = 0b0100000100100111;

// Current state machine state
uint32_t current_state = STATE_INIT;

uint8_t led1_value = 0;
uint8_t led2_value = 0;

// Board control values
uint8_t board_hs[4] = { 0, 0, 0, 0 }; // Connected board slots
float board_voltages[4] = { 0.0f, 0.0f, 0.0f, 0.0f };  // C9, C10_3, C10_2, C10_1
float board_currents[4] = { 0.0f, 0.0f, 0.0f, 0.0f };  // C9, C10_3, C10_2, C10_1
float board_powers[4] = { 0.0f, 0.0f, 0.0f, 0.0f };    // C9, C10_3, C10_2, C10_1
float batt_voltage = 0.0f;
float batt_current = 0.0f;
float batt_power = 0.0f;
float volant_voltage = 0.0f;
float volant_current = 0.0f;
float volant_power = 0.0f;

uint8_t volant_status = 1;  // 1 = no error,  0 = error

// Feedback values of GPIOs
uint8_t feedback_gpio0_gp0;
uint8_t feedback_gpio0_gp1;
uint8_t feedback_gpio1_gp0;
uint8_t feedback_gpio1_gp1;
uint8_t feedback_status_24V;

uint8_t can_error = 0;

uint8_t b_timer500ms_flag;
uint8_t b_timer100ms_flag;
uint8_t b_timer200ms_flag;


// TEMP TEMP
uint8_t pb1_update;
uint8_t pb2_update;
uint8_t pb1_value;
uint8_t pb2_value;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

// State machine control
void ExecuteStateMachine();

uint32_t DoStateInit();
uint32_t DoStateBoardControl();
uint32_t DoStateAcquisition();
uint32_t DoStateCan();

void DoStateError();

HAL_StatusTypeDef GPIO_SendI2C(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t GPIO_ReadI2C(uint8_t addr, uint8_t reg);

HAL_StatusTypeDef INA_SendI2C(uint8_t addr, uint8_t reg, uint16_t data);
uint16_t INA_ReadI2C(uint8_t addr, uint8_t reg);

// GPIOs

void EnableAllVoltages();

void InitGPIOs();

void DisableAllVoltages();
void EnableVoltage(uint8_t board, uint8_t voltage);
void DisableVoltage(uint8_t board, uint8_t voltage);

void EnableAllBoardVoltages(uint8_t board);
void DisableAllBoardVoltages(uint8_t board);

void EnableVolant24V();
void DisableVolant24V();

void SetLed(uint8_t led, uint8_t value);
void ToggleLed(uint8_t led);

void BeepSpeaker();

HAL_StatusTypeDef SendUART(uint8_t* buffer, size_t size);
void EnableFT230Tx();
void DisableFT230Tx();
void ResetFT230();

void InitAllINA226s();

float ReadShuntVoltageINA226(uint8_t ina226_id);
float ReadBusVoltageINA226(uint8_t ina226_id);
float ReadCurrentINA226(uint8_t ina226_id);
float ReadPowerINA226(uint8_t ina226_id);


// CAN

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void delay_us(uint16_t delay16_us)
{
	htim1.Instance->CNT = 0;
	while (htim1.Instance->CNT < delay16_us);
}

void delay_ms(uint16_t delay16_ms)
{
	while(delay16_ms > 0)
	{
		htim1.Instance->CNT = 0;
		delay16_ms--;
		while (htim1.Instance->CNT < 1000);
	}
}


HAL_StatusTypeDef GPIO_SendI2C(uint8_t addr, uint8_t reg, uint8_t data)
{
  uint8_t buf[2];

  buf[0] = reg;
  buf[1] = data;

  return HAL_I2C_Master_Transmit(&hi2c1, addr, buf, sizeof(buf), HAL_MAX_DELAY);
}

uint8_t GPIO_ReadI2C(uint8_t addr, uint8_t reg)
{
	HAL_StatusTypeDef ret;

	uint8_t cmd[1];
	cmd[0] = reg;

	ret = HAL_I2C_Master_Transmit(&hi2c1, addr, cmd, sizeof(cmd), HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	uint8_t buf[1];
	ret = HAL_I2C_Master_Receive(&hi2c1, addr, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	return buf[0];
}

HAL_StatusTypeDef INA_SendI2C(uint8_t addr, uint8_t reg, uint16_t data)
{
	uint8_t buf[3];

	buf[0] = reg;
	buf[1] = (data >> 8) & 0xFF;
	buf[2] = data & 0xFF;

	return HAL_I2C_Master_Transmit(&hi2c3, addr, buf, sizeof(buf), HAL_MAX_DELAY);
}

uint16_t INA_ReadI2C(uint8_t addr, uint8_t reg)
{
	HAL_StatusTypeDef ret;

	uint8_t cmd[1];
	cmd[0] = reg;

	ret = HAL_I2C_Master_Transmit(&hi2c3, addr, cmd, sizeof(cmd), HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	// Reading 16bits for INA
	uint8_t buf[2];
	ret = HAL_I2C_Master_Receive(&hi2c3, addr, buf, sizeof(buf), HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	// return buf[0];
	uint16_t result = (buf[0] << 8) | buf[1];
	return result;
}

void InitGPIOs()
{
  //
  // Default values for GPIO registers
  //

  // GPIO directions
  gpio0_dir0 = 0;
  gpio0_dir1 = 0b11110000;
  gpio1_dir0 = 0;
  gpio1_dir1 = 0;

  // GPIO *input* polarity
  gpio0_pol0 = 0;
  gpio0_pol1 = 0;
  gpio1_pol0 = 0;
  gpio1_pol1 = 0;

  // GPIO reset to 0
  gpio0_gp0 = gpio0_gp1 = 0;
  // Enable 3V3 and 5V have inverted enable logic (1 = disable for 5V and 3V3)
  gpio1_gp0 = gpio1_gp1 = 0b11001100;

  //
  // Send values to GPIO
  //

  //HAL_StatusTypeDef ret;
  //uint8_t buf[12];

  uint8_t error = 0;
  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_DIR_GP0, gpio0_dir0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_DIR_GP1, gpio0_dir1);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_DIR_GP0, gpio1_dir0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_DIR_GP1, gpio1_dir1);
  delay_us(5);

  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_POL_GP0, gpio0_pol0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_POL_GP1, gpio0_pol1);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_POL_GP0, gpio1_pol0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_POL_GP1, gpio1_pol1);
  delay_us(5);

  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_GP0, gpio0_gp0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO0_ADDR, GPIO_GP1, gpio0_gp1);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
  delay_us(5);
  error |= GPIO_SendI2C(GPIO1_ADDR, GPIO_GP1, gpio1_gp1);
  delay_us(5);

  if (error)
  {
	  // TODO
  }

}

void DisableAllVoltages()
{
	gpio1_gp0 = 0b11001100;
	gpio1_gp1 = 0b11001100;
	GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
	delay_ms(10);
	GPIO_SendI2C(GPIO1_ADDR, GPIO_GP1, gpio1_gp1);
	delay_ms(10);

	// Disable Volant 24V
	DisableVolant24V();
	delay_ms(10);
}

// board = 1,2,3,4
//  0,1,2 = Board C10 1,2,3
//  3     = Board C9
void EnableVoltage(uint8_t board, uint8_t voltage)
{
	// Special case for board C9 - 24V,15V
	if (board == BOARD_C9 && voltage == VOLTAGE_24V)
	{
		gpio1_gp0 |= (1 << 1);
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
		return;
	}

	unsigned value = board * 4 + voltage;
	unsigned gp = value / (unsigned)8;
	if (gp)
	{
		// Inverted logic for 5V and 3V3
		if (voltage == VOLTAGE_5V || voltage == VOLTAGE_3V3)
			gpio1_gp1 &= ~(1 << (value - 8));
		else
			gpio1_gp1 |= (1 << (value - 8));
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP1, gpio1_gp1);
	}
	else
	{
		// Inverted logic for 5V and 3V3
		if (voltage == VOLTAGE_5V || voltage == VOLTAGE_3V3)
			gpio1_gp0 &= ~(1 << value);
		else
			gpio1_gp0 |= (1 << value);
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
	}
}

void DisableVoltage(uint8_t board, uint8_t voltage)
{
	// Special case for board C9 - 24V,15V
	if (board == BOARD_C9 && voltage == VOLTAGE_24V)
	{
		gpio1_gp0 &= ~(1 << 1);
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
		return;
	}

	unsigned value = board * 4 + voltage;
	unsigned gp = value / (unsigned)8;
	if (gp)
	{
		// Inverted logic for 5V and 3V3
		if (voltage == VOLTAGE_5V || voltage == VOLTAGE_3V3)
			gpio1_gp1 |= (1 << (value - 8));
		else
			gpio1_gp1 &= ~(1 << (value - 8));
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP1, gpio1_gp1);
	}
	else
	{
		// Inverted logic for 5V and 3V3
		if (voltage == VOLTAGE_5V || voltage == VOLTAGE_3V3)
			gpio1_gp0 |= (1 << value);
		else
			gpio1_gp0 &= ~(1 << value);
		GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
	}
}

void EnableAllBoardVoltages(uint8_t board)
{
	EnableVoltage(board, VOLTAGE_3V3);
	EnableVoltage(board, VOLTAGE_5V);
	EnableVoltage(board, VOLTAGE_24V);
}

void DisableAllBoardVoltages(uint8_t board)
{
	DisableVoltage(board, VOLTAGE_3V3);
	DisableVoltage(board, VOLTAGE_5V);
	DisableVoltage(board, VOLTAGE_24V);
}

void EnableVolant24V()
{
	HAL_GPIO_WritePin(Volant_Enable24V_GPIO_Port, Volant_Enable24V_Pin, 1);
}

void DisableVolant24V()
{
	HAL_GPIO_WritePin(Volant_Enable24V_GPIO_Port, Volant_Enable24V_Pin, 0);
}

void SetLed(uint8_t led, uint8_t value)
{
	if (led >= NUM_LEDS)
		return;

	if (value)
		gpio0_gp0 |= (1 << led);
	else
		gpio0_gp0 &= ~(1 << led);

	GPIO_SendI2C(GPIO0_ADDR, GPIO_GP0, gpio0_gp0);
}

void ToggleLed(uint8_t led)
{
	if (led >= NUM_LEDS)
		return;

	uint8_t value = (gpio0_gp0 & (1 << led)) >> led;
	SetLed(led, !value);
}

// CAN Rx Callback
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, rxData) != HAL_OK)
	{
		Error_Handler();
	}

	if (pRxHeader.StdId == 0xAA)
	{

	}
	else if (pRxHeader.StdId == 0xBB)
	{

	}
	// etc etc ..
}
*/

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size)
{
	// CAN_TxHeaderTypeDef msg;

	pTxHeader.StdId = id;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.DLC = size; // Number of bytes to send
	pTxHeader.TransmitGlobalTime = DISABLE;

	uint32_t mb;
	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
	if (ret != HAL_OK)
	{
		// SetLed(LED_CAN, 1);
		SetLed(LED_ERROR, 1);
		return ret;
	}

	// Update the CAN led
	//ToggleLed(LED_CAN);
	// SetLed(LED_CAN, 0);

	return ret;

	return 0;
}

void BeepSpeaker()
{
	HAL_GPIO_WritePin(Speaker_GPIO_Port, Speaker_Pin, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Speaker_GPIO_Port, Speaker_Pin, 0);
}

HAL_StatusTypeDef SendUART(uint8_t* buffer, size_t size)
{
	return HAL_UART_Transmit(&huart2, buffer, size, HAL_MAX_DELAY);
}

void EnableFT230Tx()
{
	HAL_GPIO_WritePin(UART_TXDEn_GPIO_Port, UART_TXDEn_Pin, 1);
}
void DisableFT230Tx()
{
	HAL_GPIO_WritePin(UART_TXDEn_GPIO_Port, UART_TXDEn_Pin, 0);
}
void ResetFT230()
{
	HAL_GPIO_WritePin(FT230_RESET_GPIO_Port, FT230_RESET_Pin, 1);
	HAL_Delay(2);
	HAL_GPIO_WritePin(FT230_RESET_GPIO_Port, FT230_RESET_Pin, 0);
}

void InitAllINA226s()
{
	// Configuration register
	// AVG mode set to 4 averages (everything else is default)
	uint16_t config_reg_value = 0x4327;
	for (int i = 0; i < NUM_INA226; ++i)
	{
		INA_SendI2C(INA226_ADDRs[i], INA226_CONFIG_REG, config_reg_value);
	}

	// Calibration register
	typedef union
	{
		float cal_f;
		uint32_t cal_i;
	} CalPunning;
	CalPunning cal_punning;

	cal_punning.cal_f = 0.00052f / INA226_CURRENT_LSB * INA226_RSHUNT;
	uint16_t cal_reg_value = (uint16_t)(cal_punning.cal_i & 0x0000FFFF);

	for (int i = 0; i < NUM_INA226; ++i)
	{
		INA_SendI2C(INA226_ADDRs[i], INA226_CALIB_REG, cal_reg_value);
	}
}

float ReadShuntVoltageINA226(uint8_t ina226_id)
{
	uint16_t shunt_voltage = INA_ReadI2C(INA226_ADDRs[ina226_id], INA226_BUS_VOLTAGE_REG);
	return ((float)shunt_voltage) * INA226_SHUNT_VOLTAGE_LSB;
}

float ReadBusVoltageINA226(uint8_t ina226_id)
{
	uint16_t bus_voltage = INA_ReadI2C(INA226_ADDRs[ina226_id], INA226_BUS_VOLTAGE_REG);
	return ((float)bus_voltage) * INA226_BUS_VOLTAGE_LSB;
}

float ReadCurrentINA226(uint8_t ina226_id)
{
	uint16_t current = INA_ReadI2C(INA226_ADDRs[ina226_id], INA226_CURRENT_REG);
	return ((float)current) * INA226_CURRENT_LSB;
}

float ReadPowerINA226(uint8_t ina226_id)
{
	uint16_t power = INA_ReadI2C(INA226_ADDRs[ina226_id], INA226_POWER_REG);
	return ((float)power) * INA226_POWER_LSB;
}

void ExecuteStateMachine()
{
	// Blink LED to signify alive and working
	if (b_timer500ms_flag)
	{
		b_timer500ms_flag = 0;

		timer_flag = 1;
	}
	if (b_timer200ms_flag)
	{
		b_timer200ms_flag = 0;

		b_acquisition_gpios_flag = 1;
	}
	if (b_timer100ms_flag)
	{
		b_timer100ms_flag = 0;

		b_flag_can_tx = 1;
	}

	if (timer_flag)
	{
		timer_flag = 0;

		ToggleLed(LED1);
	}

	switch (current_state)
	{
	case STATE_INIT:
		current_state = DoStateInit();
		break;

	case STATE_ACQUISITION:
		current_state = DoStateAcquisition();
		break;

	case STATE_BOARD_CONTROL:
		current_state = DoStateBoardControl();
		break;

	case STATE_CAN:
		current_state = DoStateCan();
		break;

	case STATE_ERROR:
		DoStateError();
		// In case we escape the error handler, re-initialize
		current_state = DoStateInit();
		break;

	default:
		DoStateInit();
		break;
	};
}

uint32_t DoStateInit()
{
	// Init global variables
	timer_flag = 0;
	b_acquisition_gpios_flag = 0;
	b_flag_can_tx = 0;

	pb1_pressed = 0;
	pb2_pressed = 0;

	can1_recv_flag = 0;

	volant_status = 1;
	can_error = 0;

	b_timer500ms_flag = 0;
	b_timer100ms_flag = 0;
	b_timer200ms_flag = 0;

	// Initialize GPIO values
	InitGPIOs();
	HAL_Delay(10);

	// DisableAllVoltages();
	HAL_Delay(10);

	InitAllINA226s();

	delay_ms(10);
	EnableAllVoltages();
	delay_ms(10);

	// Start by verifying values of GPIOs
	delay_ms(5);
	b_acquisition_gpios_flag = 1;

	return STATE_ACQUISITION;
}

uint32_t DoStateAcquisition()
{
	// Read HS signals and enable voltages according to slots used
	board_hs[3] = HAL_GPIO_ReadPin(HS1_GPIO_Port, HS1_Pin);  // Board C10_1
	board_hs[2] = HAL_GPIO_ReadPin(HS2_GPIO_Port, HS2_Pin);  // Board C10_2
	board_hs[1] = HAL_GPIO_ReadPin(HS3_GPIO_Port, HS3_Pin);  // Board C10_3
	board_hs[0] = HAL_GPIO_ReadPin(HS4_GPIO_Port, HS4_Pin);  // Board C9

	// Read INA226 voltage and current values
	for (int i = 0; i < 4; ++i)
	{
		board_voltages[i] = ReadBusVoltageINA226(i);
		board_currents[i] = ReadCurrentINA226(i);
		board_powers[i] = ReadPowerINA226(i);
	}

	batt_voltage = ReadBusVoltageINA226(BATT_INA);
	batt_current = ReadCurrentINA226(BATT_INA);
	batt_power = ReadPowerINA226(BATT_INA);

	volant_voltage = ReadBusVoltageINA226(VOLANT_INA);
	volant_current = ReadCurrentINA226(VOLANT_INA);
	volant_power = ReadPowerINA226(VOLANT_INA);


	volant_status = HAL_GPIO_ReadPin(Volant_Status_GPIO_Port, Volant_Status_Pin);

	// Verify values of GPIO expanders
	if (b_acquisition_gpios_flag) // Executed every 200ms
	{
		b_acquisition_gpios_flag = 0;

		/*
		feedback_gpio0_gp0 = GPIO_ReadI2C(GPIO0_ADDR, GPIO_GP0);
		delay_us(10);
		feedback_gpio0_gp1 = GPIO_ReadI2C(GPIO0_ADDR, GPIO_GP1);
		delay_us(10);
		feedback_gpio1_gp0 = GPIO_ReadI2C(GPIO1_ADDR, GPIO_GP0);
		delay_us(10);
		feedback_gpio1_gp1 = GPIO_ReadI2C(GPIO1_ADDR, GPIO_GP1);
		delay_us(10);

		if (feedback_gpio0_gp0 != gpio0_gp0)
		{
			GPIO_SendI2C(GPIO0_ADDR, GPIO_GP0, gpio0_gp0);
			delay_us(10);
		}

		if (feedback_gpio1_gp0 != gpio1_gp0)
		{
			GPIO_SendI2C(GPIO1_ADDR, GPIO_GP0, gpio1_gp0);
			delay_us(10);
		}
		if (feedback_gpio1_gp1 != gpio1_gp1)
		{
			GPIO_SendI2C(GPIO1_ADDR, GPIO_GP1, gpio1_gp1);
			delay_us(10);
		}

		// Verify 24V status
		feedback_status_24V = (feedback_gpio0_gp1 & 0xF0) >> 4;
		*/


		/*
		// Status at low = error on 24V
		if ((feedback_status_24V & 0x1) == 0)
		{
			DisableVoltage(BOARD_C9, VOLTAGE_24V);
			delay_us(10);
			EnableVoltage(BOARD_C9, VOLTAGE_24V);
		}
		if ((feedback_status_24V & 0x2) == 0)
		{
			DisableVoltage(BOARD_C10_3, VOLTAGE_24V);
			delay_us(10);
			EnableVoltage(BOARD_C10_3, VOLTAGE_24V);
		}
		if ((feedback_status_24V & 0x4) == 0)
		{
			DisableVoltage(BOARD_C10_2, VOLTAGE_24V);
			delay_us(10);
			EnableVoltage(BOARD_C10_2, VOLTAGE_24V);
		}
		if ((feedback_status_24V & 0x8) == 0)
		{
			DisableVoltage(BOARD_C10_1, VOLTAGE_24V);
			delay_us(10);
			EnableVoltage(BOARD_C10_1, VOLTAGE_24V);
		}
		*/

	}

	return STATE_BOARD_CONTROL;
}

uint32_t DoStateBoardControl()
{
	// Enable or disable voltages based on HS board slots values
	/*
	if (board_hs[0])
	{
		EnableVoltage(BOARD_C9, VOLTAGE_3V3);
		EnableVoltage(BOARD_C9, VOLTAGE_5V);
		EnableVoltage(BOARD_C9, VOLTAGE_24V);
	}
	*/

	// TODO: (Marc) Don't resend every time, check if value has changed
	// EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
	// EnableVoltage(BOARD_C10_2, VOLTAGE_5V);
	// EnableVoltage(BOARD_C10_2, VOLTAGE_24V);

	// Check that the values of GPIO expanders match expected values



	return STATE_CAN;
}


uint32_t DoStateCan()
{
	if (b_flag_can_tx)
	{
		b_flag_can_tx = 0;

		static uint32_t dummy_data_mario = 0;
		++dummy_data_mario;
		TransmitCAN(BACKPLANE_DUMMY_TRAFFIC_MARIO, (uint8_t*)&dummy_data_mario, 4);
		delay_us(10);

		static uint32_t dummy_data_drive = 10;
		++dummy_data_drive;
		TransmitCAN(BACKPLANE_DUMMY_TRAFFIC_DRIVE, (uint8_t*)&dummy_data_drive, 4);
		delay_us(10);

		// Send all sensor values
		// HAL_StatusTypeDef can_success = HAL_OK;

		/*
		can_success &= TransmitCAN(BACKPLANE_TOTAL_VOLTAGE, (uint8_t*)(&batt_voltage), 4);
		can_success &= TransmitCAN(BACKPLANE_TOTAL_CURRENT, (uint8_t*)(&batt_current), 4);

		can_success &= TransmitCAN(BACKPLANE_VOLANT_VOLTAGE, (uint8_t*)(&volant_voltage), 4);
		can_success &= TransmitCAN(BACKPLANE_VOLANT_CURRENT, (uint8_t*)(&volant_current), 4);

		float board_voltages_1[2] = { board_voltages[0], board_voltages[1] };
		can_success &= TransmitCAN(BACKPLANE_BOARD_VOLTAGES_1, (uint8_t*)(&board_voltages_1), 8);
		float board_voltages_2[2] = { board_voltages[2], board_voltages[3] };
		can_success &= TransmitCAN(BACKPLANE_BOARD_VOLTAGES_2, (uint8_t*)(&board_voltages_2), 8);

		float board_currents_1[2] = { board_currents[0], board_currents[1] };
		can_success &= TransmitCAN(BACKPLANE_BOARD_CURRENTS_1, (uint8_t*)(&board_currents_1), 8);
		float board_currents_2[2] = { board_currents[2], board_currents[3] };
		can_success &= TransmitCAN(BACKPLANE_BOARD_CURRENTS_2, (uint8_t*)(&board_currents_2), 8);

		can_success &= TransmitCAN(BACKPLANE_BOARD_SLOTS_HS, board_hs, 4);

		can_success &= TransmitCAN(BACKPLANE_VOLANT_STATUS, &volant_status, 1);
		*/


		/*
		if (can_success != HAL_OK)
		{
			SetLed(LED_ERROR, 1);
			can_error = 1;
		}
		else
		{
			SetLed(LED_ERROR, 0);
			can_error = 0;
		}
		*/
	}

	return STATE_ACQUISITION;
}

void DoStateError()
{
	Error_Handler();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	typedef union RxToInt_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
	} RxToInt;
	static RxToInt rxToInt;

	if (hcan->Instance == CAN1)
	{
		// Indicate CAN working with CAN led
		//HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
		ToggleLed(LED_CAN);

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, rxData) != HAL_OK)
		{
			Error_Handler();
		}

		uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);

		if (pRxHeader.StdId == MARIO_PITCH_MANUAL_CMD) // 0x71
		{
			if (can_data == 0x300)
			{
				SetLed(LED3, 1);
			}
			else if (can_data == 0x200)
			{
				SetLed(LED4, 1);
			}
			else if (can_data == 0x100)
			{
				SetLed(LED3, 0);
				SetLed(LED4, 0);
			}
		}
		else if (pRxHeader.StdId == MARIO_BUZZER_CMD) // 0x81
		{
			ToggleLed(LED2);
		}
		/*
		else if (pRxHeader.StdId == MARIO_PITCH_MODE_CMD)
		{
			uint8_t pitch_mode = rxData[0];
		}
		else if (pRxHeader.StdId == MARIO_MAST_MODE_CMD)
		{
			uint8_t mast_mode = rxData[0];
		}
		else
		{
			// Unknown CAN ID
		}
		*/
	}
	else
	{
		// Wrong CAN interface
	}
}

void EnableAllVoltages()
{
	EnableVoltage(BOARD_C10_1, VOLTAGE_3V3);
	delay_ms(5);
	EnableVoltage(BOARD_C10_1, VOLTAGE_5V);
	delay_ms(5);
	EnableVoltage(BOARD_C10_1, VOLTAGE_24V);
	delay_ms(5);

	EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
	delay_ms(5);
	EnableVoltage(BOARD_C10_2, VOLTAGE_5V);
	delay_ms(5);
	EnableVoltage(BOARD_C10_2, VOLTAGE_24V);
	delay_ms(5);

	EnableVoltage(BOARD_C10_3, VOLTAGE_3V3);
	delay_ms(5);
	EnableVoltage(BOARD_C10_3, VOLTAGE_5V);
	delay_ms(5);
	EnableVoltage(BOARD_C10_3, VOLTAGE_24V);
	delay_ms(5);

	EnableVoltage(BOARD_C9, VOLTAGE_3V3);
	delay_ms(5);
	EnableVoltage(BOARD_C9, VOLTAGE_5V);
	delay_ms(5);
	EnableVoltage(BOARD_C9, VOLTAGE_24V);
	delay_ms(5);

	EnableVolant24V();
	delay_ms(5);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);

  /*EnableVoltage(BOARD_C10_1, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_3, VOLTAGE_3V3);

  EnableVoltage(BOARD_C10_1, VOLTAGE_5V);
  EnableVoltage(BOARD_C10_2, VOLTAGE_5V);
  EnableVoltage(BOARD_C10_3, VOLTAGE_5V);*/
/*
  EnableVoltage(BOARD_C9, VOLTAGE_3V3);
  EnableVoltage(BOARD_C9, VOLTAGE_5V);
  EnableVoltage(BOARD_C9, VOLTAGE_24V);

  EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_2, VOLTAGE_5V);
  EnableVoltage(BOARD_C10_2, VOLTAGE_24V);
*/
  // EnableVoltage(BOARD_C10_1, VOLTAGE_15V);
  // EnableVoltage(BOARD_C10_2, VOLTAGE_15V);
  // EnableVoltage(BOARD_C10_3, VOLTAGE_15V);

  // EnableVoltage(BOARD_C10_1, VOLTAGE_24V);
  // EnableVoltage(BOARD_C10_2, VOLTAGE_24V);
  // EnableVoltage(BOARD_C10_3, VOLTAGE_24V);

  /*
  EnableVoltage(BOARD_C10_1, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_1, VOLTAGE_5V);
  EnableVoltage(BOARD_C10_1, VOLTAGE_24V);

  EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_2, VOLTAGE_5V);

  EnableVoltage(BOARD_C10_3, VOLTAGE_3V3);
  EnableVoltage(BOARD_C10_3, VOLTAGE_5V);

  // Enable 24V volant
  EnableVolant24V();
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  DoStateInit();
  delay_ms(5);

  EnableVoltage(BOARD_C10_1, VOLTAGE_3V3);
  delay_ms(5);
  EnableVoltage(BOARD_C10_1, VOLTAGE_5V);
  delay_ms(5);
  EnableVoltage(BOARD_C10_1, VOLTAGE_24V);
  delay_ms(5);

  EnableVoltage(BOARD_C10_2, VOLTAGE_3V3);
  delay_ms(5);
  EnableVoltage(BOARD_C10_2, VOLTAGE_5V);
  delay_ms(5);
  EnableVoltage(BOARD_C10_2, VOLTAGE_24V);
  delay_ms(5);

  EnableVoltage(BOARD_C10_3, VOLTAGE_3V3);
  delay_ms(5);
  EnableVoltage(BOARD_C10_3, VOLTAGE_5V);
  delay_ms(5);
  EnableVoltage(BOARD_C10_3, VOLTAGE_24V);
  delay_ms(5);

  EnableVoltage(BOARD_C9, VOLTAGE_3V3);
  delay_ms(5);
  EnableVoltage(BOARD_C9, VOLTAGE_5V);
  delay_ms(5);
  EnableVoltage(BOARD_C9, VOLTAGE_24V);
  delay_ms(5);

  EnableVolant24V();
  delay_ms(5);


  current_state = STATE_INIT;
  while (1)
  {
	  ExecuteStateMachine();

	  // TODO: (Marc) Should be replaced by a timer with better resolution
	  HAL_Delay(5);
  }


  uint8_t buf1[4] = {0};
  uint8_t buf2[4] = {0};
  uint32_t buf1_val = 0x200;
  uint32_t buf2_val = 0x1;
  memcpy(buf1, &buf1_val, 4);
  memcpy(buf2, &buf2_val, 4);

  uint8_t buf[4];
  uint32_t dir_stop = MOTOR_DIRECTION_STOP;
  uint32_t dir_left = MOTOR_DIRECTION_LEFT;
  uint32_t dir_right = MOTOR_DIRECTION_RIGHT;

  while (1)
  {
	  HAL_Delay(10);

	  // Alive blink led
	  if (b_timer500ms_flag == 1)
	  {
		  b_timer500ms_flag = 0;

		  ToggleLed(LED1);

		  // Making sure ?
		  // EnableAllVoltages();
	  }

	  /*
	  uint8_t buf[4];

	  if(GPIO_PIN_SET == HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin)) // PD_14 -- PB2
	  {
		//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		  if (pb2_value == 1)
		  {
			  pb2_value = 0;
			  // HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			  SetLed(LED2, 0);

			  memcpy(buf, &dir_stop, 4);
			  TransmitCAN(0x71, buf, 4);
		  }
	  }
	  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin)) // PD_15 -- PB1
	  {
		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		  if (pb1_value == 1)
		  {
			  pb1_value = 0;
			  // HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			  SetLed(LED3, 0);

			  memcpy(buf, &dir_stop, 4);
			  TransmitCAN(0x71, buf, 4);
		  }
	  }

	  if (pb1_update)
	  {
		  memcpy(buf, &dir_left, 4);
		  TransmitCAN(0x71, buf, 4);
		  pb1_update = 0;
	  }
	  if (pb2_update)
	  {
		  memcpy(buf, &dir_right, 4);
		  TransmitCAN(0x71, buf, 4);
		  pb2_update = 0;
	  }
	  */
  }

  while (1)
  {
	  // Alive blink led
	  if (b_timer500ms_flag == 1)
	  {
		  b_timer500ms_flag = 0;

		  ToggleLed(LED1);
	  }

	  // PD_14 -- PB2
	  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin))
	  {
		  //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		  if (led1_value == 1)
		  {
			  led1_value = 0;
			  SetLed(LED1, 0);
		  }
	  }
	  // PD_15 -- PB1
	  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin))
	  {
		  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  if (led2_value == 1)
		  {
			  led2_value = 0;
			  SetLed(LED2, 0);
		  }
	  }



	  if (led1_value == 1)
	  {
		  //TransmitCAN(0xAA, buf1, sizeof(buf1));
	  }
	  else if (led2_value == 1)
	  {
		  //TransmitCAN(0xAA, buf2, sizeof(buf2));
	  }
	  else
	  {
		  static uint8_t buf0[4] = {0};
		  //TransmitCAN(0xAA, buf0, sizeof(buf0));
	  }

	  HAL_Delay(10);
  }

  //HAL_StatusTypeDef ret;

  while (1)
  {
	  if (timer_flag)
	  {
		  timer_flag = 0;

		  // Blink led warning as living-led
		  ToggleLed(LED_WARN);

		  // Send CAN frame
		  /*
		  static uint8_t x = 0;
		  uint8_t data[4] = { 0,1,2,x++ };
		  HAL_StatusTypeDef can_success = TransmitCAN(0xAA, data, 4);
		  if (can_success != HAL_OK)
		  {
			  SetLed(LED_ERROR, 1);
		  }
		  else
		  {
			  SetLed(LED_ERROR, 0);
		  }
		  */

		  // Send UART Message
		  uint8_t msg[] = "Hello World!";
		  HAL_StatusTypeDef uart_success = SendUART(msg, sizeof(msg));
		  if (uart_success != HAL_OK)
		  {
			  // TODO
			  // ErrorHandler();
		  }
	  }

	  // Check which boards are connected
	  // uint8_t hs1 = HAL_GPIO_ReadPin(HS1_GPIO_Port, HS1_Pin);  // Board C10_1
	  // uint8_t hs2 = HAL_GPIO_ReadPin(HS2_GPIO_Port, HS2_Pin);  // Board C10_2
	  // uint8_t hs3 = HAL_GPIO_ReadPin(HS3_GPIO_Port, HS3_Pin);  // Board C10_3
	  // uint8_t hs4 = HAL_GPIO_ReadPin(HS4_GPIO_Port, HS4_Pin);  // Board C9
	  // Update 4 leds based on connection status
	  //SetLed(LED1, hs1);
	  //SetLed(LED2, hs2);
	  //SetLed(LED3, hs3);
	  //SetLed(LED4, hs4);

	  // Get Volant BTS428L2 status (low on error)
	  // uint8_t volant_status = HAL_GPIO_ReadPin(Volant_Status_GPIO_Port, Volant_Status_Pin);
	  /*
	  if (volant_status == 0)
	  {
		  // Disable volant voltage
		  DisableVolant24V();
		  // TODO: Error management
		  // SetLed(LED_ERROR, 1);
	  }
	  */

	  // Get Battery IMON value
	  /*
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uint16_t raw_imon = HAL_ADC_GetValue(&hadc1);
	  */

	  // Check messages received on CANA bus
	  if (can1_recv_flag)
	  {
		  can1_recv_flag = 0;

		  uint32_t id = pRxHeader.StdId;
	  }

	  // Execute main loop every 10ms
	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  // HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

/*
  CAN_FilterTypeDef filter_all;
  	// All common bits go into the ID register
  filter_all.FilterIdHigh = 0x0000;
  filter_all.FilterIdLow = 0x0000;

  	// Which bits to compare for filter
  filter_all.FilterMaskIdHigh = 0x0000;
  filter_all.FilterMaskIdLow = 0x0000;

  filter_all.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter_all.FilterBank = 2; // Which filter to use from the assigned ones
  filter_all.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_all.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_all.FilterActivation = CAN_FILTER_ENABLE;
  filter_all.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
  	if (HAL_CAN_ConfigFilter(&hcan1, &filter_all) != HAL_OK)
  	{
  	  Error_Handler();
  	}
  	*/




    CAN_FilterTypeDef sf_fifo0;
	// All common bits go into the ID register
	sf_fifo0.FilterIdHigh = BACKPLANE_FIFO0_RX_FILTER_ID_HIGH;
	sf_fifo0.FilterIdLow = BACKPLANE_FIFO0_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_fifo0.FilterMaskIdHigh = BACKPLANE_FIFO0_RX_FILTER_MASK_HIGH;
	sf_fifo0.FilterMaskIdLow = BACKPLANE_FIFO0_RX_FILTER_MASK_LOW;

	sf_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_fifo0.FilterBank = 0; // Which filter to use from the assigned ones
	sf_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo0.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo0.SlaveStartFilterBank = 14; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo0) != HAL_OK)
	{
	  Error_Handler();
	}

/*
	CAN_FilterTypeDef sf_fifo1;
		// All common bits go into the ID register
	sf_fifo1.FilterIdHigh = 0xFFFF;
	sf_fifo1.FilterIdLow = 0xFFFF;

		// Which bits to compare for filter
	sf_fifo1.FilterMaskIdHigh = 0xFFFF;
	sf_fifo1.FilterMaskIdLow = 0xFFFF;

	sf_fifo1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_fifo1.FilterBank = 1; // Which filter to use from the assigned ones
	sf_fifo1.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo1.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo1.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo1.SlaveStartFilterBank = 14; // How many filters to assign to CAN1
		if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo1) != HAL_OK)
		{
		  Error_Handler();
		}
*/

  	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}


  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  // Start the timer
  HAL_TIM_Base_Start(&htim1);

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 480;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 480;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Speaker_GPIO_Port, Speaker_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_PWR_RPI_Pin|CLK_PWR_RPI_Pin|SHUTDOWN_RPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Volant_Enable24V_Pin|UART_TXDEn_Pin|FT230_RESET_Pin|USB_PROG_EN_Pin
                          |USB_Enable1_Pin|USB_Enable2_Pin|USB_Enable3_Pin|USB_Enable4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Speaker_Pin */
  GPIO_InitStruct.Pin = Speaker_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Speaker_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_PWR_RPI_Pin CLK_PWR_RPI_Pin SHUTDOWN_RPI_Pin */
  GPIO_InitStruct.Pin = EN_PWR_RPI_Pin|CLK_PWR_RPI_Pin|SHUTDOWN_RPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RUNNING_RPI_Pin DETECT_PWR_RPI_Pin HS1_Pin HS2_Pin
                           HS3_Pin HS4_Pin */
  GPIO_InitStruct.Pin = RUNNING_RPI_Pin|DETECT_PWR_RPI_Pin|HS1_Pin|HS2_Pin
                          |HS3_Pin|HS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Volant_Enable24V_Pin UART_TXDEn_Pin FT230_RESET_Pin USB_PROG_EN_Pin
                           USB_Enable1_Pin USB_Enable2_Pin USB_Enable3_Pin USB_Enable4_Pin */
  GPIO_InitStruct.Pin = Volant_Enable24V_Pin|UART_TXDEn_Pin|FT230_RESET_Pin|USB_PROG_EN_Pin
                          |USB_Enable1_Pin|USB_Enable2_Pin|USB_Enable3_Pin|USB_Enable4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Volant_Status_Pin INT_IO1_Pin INT_IO2_Pin */
  GPIO_InitStruct.Pin = Volant_Status_Pin|INT_IO1_Pin|INT_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_Pin PB1_Pin */
  GPIO_InitStruct.Pin = PB2_Pin|PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// EXTI Line External Interrupt ISR Handler CallBack
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // PD_14 -- PB2
    {
    	// LED4
    	//gpio0_gp0 = (gpio0_gp0 ^ (1 << 5));
    	//GPIO_SendI2C(GPIO0_ADDR, GPIO_GP0, gpio0_gp0);
    	//ToggleLed(LED1);
    	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    	SetLed(LED2, 1);
    	led1_value = 1;

    	pb2_value = 1;
    	pb2_update = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_15) // PD_15 -- PB1
    {
    	// LED3
    	//gpio0_gp0 = (gpio0_gp0 ^ (1 << 6));
    	//GPIO_SendI2C(GPIO0_ADDR, GPIO_GP0, gpio0_gp0);
    	//ToggleLed(LED2);
    	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    	SetLed(LED3, 1);
    	led2_value = 1;

    	pb1_value = 1;
    	pb1_update = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

