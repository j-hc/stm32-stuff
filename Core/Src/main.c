/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "pid.h"
#include "temp_util.h"
#include "ringbuffer.h"
#include "modishbus.h"
#include "at45db.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USE_FULL_ASSERT 1

//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//
//PUTCHAR_PROTOTYPE {
//	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
//	return ch;
//}

float pid_op;
uint16_t adc_buf[3];
uint32_t duty_cycle;
int set_point = 0;
uint16_t temp_sp_pmeter;
int internal_temp_val;
int thermistor_temp_val;
PID pid;
uint32_t clock_packed;
uint32_t date_packed;

typedef enum {
	STANDBY = 0, SET_TEMP, TEMP_STABLE, FAULT, STANDBY_IT
} State;

RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;

static int packet_ready = 0;
static int packet_msg_ready = 0;

int str2int(const char *str, int len) {
	int i = 0;
	int ret = 0;
	if (str[0] == '-')
		i++;
	for (; i < len; ++i)
		ret = ret * 10 + (str[i] - '0');
	if (str[0] == '-')
		ret *= -1;
	return ret;
}

void dev_write_slave(uint8_t *data, size_t data_size) {
// debugging print
	logd("resp -> ");
	for (size_t i = 0; i < data_size; i++)
		logd("0x%X ", data[i]);
	logd("\n");

	HAL_UART_Transmit(&huart3, data, data_size, HAL_MAX_DELAY);
}

#define NEXT_MSG_READ_MAX 32
uint8_t uart_rx_buf[FIRST_READ_SZ + NEXT_MSG_READ_MAX];

RingBuffer rb;
RegContainer rc;

#define EEPROM_PID_PAGE 0
#define PID_START_REG 1
#define CLOCK_REG 4
#define STATUS_REG 2

void r_reg_cb(Analog *regs, R_Req *req) {
	if (req->first_reg_idx == CLOCK_REG && req->reg_count == 1) {
		get_time_date(&gDate, &gTime);
		// TODO: save gdate and gtime to registers and response to this
	}
}
void w_reg_cb(Analog *regs, W_Req *req) {
// save to eeprom message
	if (req->first_reg_idx == PID_START_REG && req->reg_count == 3) {
		at45db_write_page(req->data, 3 * sizeof(uint32_t), EEPROM_PID_PAGE);
	} else if (req->first_reg_idx == CLOCK_REG && req->reg_count == 1) {
		uint32_t clock_msg = *(uint32_t*) req->data;
		set_time((clock_msg >> 2 * 8) & 0xFF, (clock_msg >> 1 * 8) & 0xFF, (clock_msg >> 0 * 8) & 0xFF);
	}
}

void handle_msg() {
	static Dev dev = { 0x1 }; // TODO: dev id?
	query(r_reg_cb, w_reg_cb, dev_write_slave, &dev, &rc, uart_rx_buf);
}

// * this requires the mode of setting EOC flag at the end of a single conversion
void ADC_ReadCH(ADC_HandleTypeDef *adcHandle, uint32_t ch, uint32_t *buf) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ch;
	sConfig.Rank = 1;
	assert(HAL_ADC_ConfigChannel(adcHandle, &sConfig) != HAL_ERROR);
	assert(HAL_ADC_Start_DMA(&hadc1, buf, 1) != HAL_ERROR);
	while (!(HAL_ADC_GetState(&hadc1) & (HAL_ADC_STATE_READY | HAL_ADC_STATE_EOC_REG)))
		__NOP();
}

void ADC_ReadCH_All(ADC_HandleTypeDef *adcHandle, uint16_t *buf, size_t size) {
	assert(size > 1);
	assert(HAL_ADC_Start_DMA(&hadc1, (uint32_t* ) buf, size) == HAL_OK);
	while (!(HAL_ADC_GetState(&hadc1) & (HAL_ADC_STATE_READY | HAL_ADC_STATE_EOC_REG)))
		__NOP();
	assert(HAL_ADC_Stop_DMA(adcHandle) == HAL_OK);
}

static uint16_t rb_buf[16];
static inline uint16_t ADC_avg_filter(RingBuffer *rb, uint16_t data) {
	rb_dequeue(rb);
	uint32_t total = 0;
	for (int i = 0; i < rb->size; i++)
		total += rb->buf[i];
	rb_queue(rb, data);
	return total / rb->size;
}

void flush_uart(void) {
	uint8_t b = 0;
	while (HAL_UART_Receive(&huart3, &b, 1, 0) != HAL_TIMEOUT)
		;
}

//static float adc_lpf_param = 0.6;
//uint32_t adc_lpf(uint32_t xi) {
//	static float y_prev = 0;
//	uint32_t y = (uint32_t) (adc_lpf_param * xi + (1 - adc_lpf_param) * y_prev);
//	y_prev = y;
//	return y;
//}

#define ARR_LEN(a) (sizeof(a)/sizeof(a[0]))

void read_temps(void) {
	//	ADC_ReadCH(&hadc1, ADC_CHANNEL_TEMPSENSOR, &temp_raw[0]);
	//	temp[0] = internal_temp(temp_raw[0]);
	//	ADC_ReadCH(&hadc1, ADC_CHANNEL_2, &temp_raw[1]);
	//	temp[1] = thermistor_temp(temp_raw[1]);

	ADC_ReadCH_All(&hadc1, adc_buf, ARR_LEN(adc_buf));
	internal_temp_val = internal_temp(adc_buf[0]);
	thermistor_temp_val = thermistor_temp(adc_buf[1]);

	return; // TODO: do i need the set point from pot-meter?
	// temp_sp_pmeter: 0 -> 100
	temp_sp_pmeter = ADC_avg_filter(&rb, adc_buf[2] * 100 / 4096);

	static uint16_t temp_sp_pmeter_prev = 0;
	if (temp_sp_pmeter != temp_sp_pmeter_prev) {
		logd("set temp (potentiometer):%d -> %d\n", set_point, temp_sp_pmeter - 50);
		set_point = temp_sp_pmeter - 50; // -50 to offset between = -50 -> +50
		temp_sp_pmeter_prev = temp_sp_pmeter;
	}
}

static State st = STANDBY;
void update_pid_pwm(float pid_op) {
	if (pid_op > 0) {
		duty_cycle = pid_op * 100.0 / pid.limMax;
		setPWMDutyCycle(&htim3, TIM_CHANNEL_3, 0);
		setPWMDutyCycle(&htim1, TIM_CHANNEL_1, duty_cycle);
	} else if (pid_op < 0) {
		duty_cycle = -(pid_op * 100.0 / pid.limMin);
		setPWMDutyCycle(&htim1, TIM_CHANNEL_1, 0);
		setPWMDutyCycle(&htim3, TIM_CHANNEL_3, duty_cycle);
	} else {
		setPWMDutyCycle(&htim3, TIM_CHANNEL_3, 0);
		setPWMDutyCycle(&htim1, TIM_CHANNEL_1, 0);
		st = TEMP_STABLE;
	}
}

#define TEMP_STABLE_OFFSET 1

void state_dispatch(void) {
	switch (st) {
	case STANDBY: {
		// HAL_PWR_EnableSleepOnExit();
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		break;
	}
	case STANDBY_IT: {
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		st = STANDBY;
		break;
	}
	case SET_TEMP: {
		read_temps();
		pid_op = PIDUpdate(&pid, set_point, thermistor_temp_val);
		update_pid_pwm(pid_op);
		break;
	}
	case TEMP_STABLE: {
		read_temps();
		if (abs(set_point - thermistor_temp_val) > TEMP_STABLE_OFFSET)
			st = SET_TEMP;
		break;
	}
	case FAULT: {
		// TODO: report error
		break;
	}
	default:
		assert(0 && "unreachable");
		break;

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();
	MX_RTC_Init();
	MX_SPI2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

// initalize PID with defaults
	{
		// PID pid = { 0 };
		pid.Kp = 1.0f;
		pid.limMin = -(float) __HAL_TIM_GET_AUTORELOAD(&htim3); // -1000
		pid.limMax = (float) __HAL_TIM_GET_AUTORELOAD(&htim1); // 1000
		pid.limMinInt = pid.limMin;
		pid.limMaxInt = pid.limMax;
		pid.T = 0.2;
	}

// initalize EEPROM
	{
		HAL_GPIO_WritePin(AT45DB_NSS_GPIO_Port, AT45DB_NSS_Pin, GPIO_PIN_SET);
		at45db_transmit_byte(AT45DB_RESUME);
		HAL_Delay(30);

		uint8_t dev_id[2] = { 0 };
		uint8_t data_devid = AT45DB_DEVID;
		at45db_transmit_receive(&data_devid, 1, dev_id, ARR_LEN(dev_id));
		// uint8_t status at45db_read_status();
		assert(dev_id[0] == 0x1f);
		assert((dev_id[1] & 0x1f) == 0x04); // at45db041d

		// restore PID coeffs
		float pid_coeffs[3] = { 0 };
		at45db_read_page((uint8_t*) pid_coeffs, 3 * sizeof(float), EEPROM_PID_PAGE);
		pid.Kp = pid_coeffs[0];
		pid.Ki = pid_coeffs[1];
		pid.Kd = pid_coeffs[2];
	}

// calibrate ADC with the ring buffer
	{
		rb = rb_init(rb_buf, ARR_LEN(rb_buf));
		for (size_t i = 0; i < ARR_LEN(rb_buf); i++) {
			ADC_ReadCH_All(&hadc1, adc_buf, ARR_LEN(adc_buf));
			int reading = adc_buf[2] * 100 / 4096;
			rb_queue(&rb, reading);
		}
	}

// set up the registers
	{
		/* AI0 -> thermistor_temp_val
		 * AI1 -> internal_temp_val */
		static uint32_t *reg_lookup_table_i[] = { (uint32_t*) &thermistor_temp_val, (uint32_t*) &internal_temp_val,
				(uint32_t*) &st };
		/* AO0 -> set_point
		 * AO1 -> PID Kp
		 * AO2 -> PID Kd
		 * AO3 -> PID Ki */

		static uint32_t *reg_lookup_table_o[] = { (uint32_t*) &set_point, (uint32_t*) &pid.Kp, (uint32_t*) &pid.Ki,
				(uint32_t*) &pid.Kd, (uint32_t*) &clock_packed };
		rc = new_reg_container(sizeof(reg_lookup_table_i), sizeof(reg_lookup_table_o), reg_lookup_table_i,
				reg_lookup_table_o);

	}

// flush uart
	{
		flush_uart();
		HAL_StatusTypeDef r = HAL_UART_Receive_IT(&huart3, uart_rx_buf, FIRST_READ_SZ);
		logd("Debug logs enabled\n");
		logd("UART Status = %d\n", r);
	}

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (packet_msg_ready) {
			handle_msg();
			packet_ready = 0;
			packet_msg_ready = 0;
			// TODO: for debugging
			memset(uart_rx_buf, 0, ARR_LEN(uart_rx_buf));
			HAL_UART_Receive_IT(&huart3, uart_rx_buf, FIRST_READ_SZ);
		}

		state_dispatch();
		HAL_Delay(200);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (packet_ready == 0) {
		size_t next_sz = next_read_sz(uart_rx_buf);
		if (next_sz > NEXT_MSG_READ_MAX) {
			logd("msg size too big: %d\n", next_sz);
			flush_uart();
			HAL_UART_Receive_IT(&huart3, uart_rx_buf, FIRST_READ_SZ);
			return;
		}
		packet_ready = 1;
		HAL_UART_Receive_IT(&huart3, uart_rx_buf + FIRST_READ_SZ, next_sz);
	} else {
		packet_msg_ready = 1;
		if (st == STANDBY)
			st = STANDBY_IT;
		HAL_ResumeTick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		HAL_ResumeTick();
		st = SET_TEMP;
	}

}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	temp[0] = internal_temp(temp_raw[0] << 4);
//	temp[1] = thermistor_temp(temp_raw[1] << 4);
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	loge("Error_Handler!\n");

	__disable_irq();
	while (1) {
	}
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	loge("assertion fail: file %s on line %lu\r\n", file, line);
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
