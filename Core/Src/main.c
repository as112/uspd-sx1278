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
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REPITER 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t sx127x_set_rx_continuos(sx127x_dev_t* dev);

sx127x_spi_t sx127x_spi = {
    .transmit = &sx127x_spi_transmit,
    .transmit_receive = &sx127x_spi_transmit_receive,
};

sx127x_callbacks_t sx127x_callbacks = {
    .rx_done = &sx127x_rx_callback,
    .tx_done = &sx127x_tx_callback,
    .rx_timeout = &sx127x_rx_timeout_callback,
    .rx_crc_error = &sx127x_rx_crc_error_callback,
};

sx127x_common_t sx127x_common = {
    .delay = &sx127x_delay,
    .reset_control = &sx127x_reset_control,
};

sx127x_dev_t sx1278_phy = {
    .spi = &sx127x_spi,
    .callbacks = &sx127x_callbacks,
    .common = &sx127x_common,
};

uint16_t countPackTrue = 0, countPackFalse = 0, countPackUp = 0, countPackDown = 0;
char str[20];
uint8_t lenn = 128;
uint8_t read_data[128];
uint8_t send_data[128];
uint8_t experement = 1;
int PN, statusRX;

union {
int in;
uint32_t u32;
} RSSI, SNR, RSSIsl, SNRsl;

union {
float fl;
uint32_t u32;
} baud_rate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
//  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
#if REPITER == 0
	lcd_Init();
  lcd_SetCursor(0, 0);
	lcd_SendString("Start");
#endif

// RF sensitivity DataSheet page 20
// Table 13 Range of Spreading Factors page 27
// Table 15 LoRa Bandwidth Options page 28

	sx127x_radio_settings_t settings = {
			.modulation = LORA,
			.mode = MODE_STDBY,
			.pa_select = PA_BOOST,
			.power = 10,
			.spreading_factor = SF_7,
			.band_width = BW_500_KHz,
			.coding_rate = CR_4_5,
			.payload_crc_on = true,
			.preamble_length = 10,
			.frequency = 433700000,				// 433050 - 434790
			.sync_word = 0x6C
	};

	sx127x_init(&sx1278_phy, &settings);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if REPITER == 1
	for(int i = 0; i < 128; i++)
		send_data[i] = 0x33;
#else
	for(int i = 0; i < 128; i++)
		send_data[i] = i;
#endif
  while (1)
  {

#if REPITER == 1
		
		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
	
		statusRX = sx127x_receive_single(&sx1278_phy, read_data, &lenn);
		if((statusRX == 0) && (experement != read_data[0])) {
			experement = read_data[0];
			changeSettingsLora(experement, settings);
			continue;
		}
		if((statusRX == 0) && (experement > 0) && (experement < 9)) {
			RSSI.in = sx127x_get_last_packet_rssi(&sx1278_phy);
			SNR.in = sx127x_get_last_packet_snr(&sx1278_phy);
			
			send_data[0] = read_data[0];
			
			send_data[4] = RSSI.u32 >> 24 & 0xFF;
			send_data[5] = (RSSI.u32 >> 16) & 0xFF;
			send_data[6] = (RSSI.u32 >> 8) & 0xFF;
			send_data[7] = RSSI.u32 & 0xFF;
			
			send_data[8] = SNR.u32 >> 24 & 0xFF;
			send_data[9] = (SNR.u32 >> 16) & 0xFF;
			send_data[10] = (SNR.u32 >> 8) & 0xFF;
			send_data[11] = SNR.u32 & 0xFF;
			HAL_Delay(20);
			sx127x_transmit(&sx1278_phy, send_data, 128, 10000);
			sx127x_set_standby(&sx1278_phy);
		}
	
#else
		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
		if (PRESS_USER_BUTTON) {
			lcd_Clearscreen();
			experement++;
			experement = (experement == 9) ? 1 : experement;
			send_data[0] = experement;
			countPackDown = 0;
			countPackUp = 0;
			sx127x_transmit(&sx1278_phy, send_data, 128, 10000);
			sx127x_set_standby(&sx1278_phy);
			changeSettingsLora(experement, settings);
			HAL_Delay(500);
			continue;
		}
		send_data[0] = experement;
		baud_rate.fl = HAL_GetTick();
		sx127x_transmit(&sx1278_phy, send_data, 128, 10000);
//		sx127x_set_standby(&sx1278_phy);
//		sx127x_set_rx_continuos(&sx1278_phy);
		baud_rate.fl = 128 * 8 * 1000 / (HAL_GetTick() - baud_rate.fl - 1);
		PN = sx127x_get_rssi(&sx1278_phy);
		countPackUp++;
	
		statusRX = sx127x_receive_continuous(&sx1278_phy, read_data, &lenn);
		
		if(statusRX == 0) {
			countPackDown++;
			
			RSSIsl.u32 = read_data[4] << 24;
			RSSIsl.u32 |= (read_data[5] << 16);
			RSSIsl.u32 |= (read_data[6] << 8);
			RSSIsl.u32 |= (read_data[7]);
		
			SNRsl.u32 = read_data[8] << 24;
			SNRsl.u32 |= (read_data[9] << 16);
			SNRsl.u32 |= (read_data[10] << 8);
			SNRsl.u32 |= (read_data[11]);
		}
		
//		if(countPackUp % 10 != 0)	continue;
		
		RSSI.in = sx127x_get_last_packet_rssi(&sx1278_phy);
		SNR.in = sx127x_get_last_packet_snr(&sx1278_phy);
		sprintf(str, "%d RSSI=%d SNR=%d ", experement, RSSI.in, SNR.in);
		
		lcd_SetCursor(0, 0);
		lcd_SendString(str);
		
		lcd_SetCursor(0, 1);
		sprintf(str, "BR=%.2f PN=%d ", baud_rate.fl, PN);
		lcd_SendString(str);
		
		lcd_SetCursor(0, 2);
		sprintf(str, "Up=%d Down=%d", countPackUp, countPackDown);
		lcd_SendString(str);
	
		lcd_SetCursor(0, 3);
		sprintf(str, "RSSI=%d SNR=%d ", RSSIsl.in, SNRsl.in);
		lcd_SendString(str);

#endif
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void changeSettingsLora(uint8_t exp, sx127x_radio_settings_t settings) {

	switch(exp) {
		case 1:
			settings.spreading_factor = SF_7;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_5;
			break;
		case 2:
			settings.spreading_factor = SF_7;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_6;
			break;
		case 3:
			settings.spreading_factor = SF_7;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_7;
			break;
		case 4:
			settings.spreading_factor = SF_7;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_8;
			break;
		case 5:
			settings.spreading_factor = SF_8;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_5;
			break;
		case 6:
			settings.spreading_factor = SF_8;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_6;
			break;
		case 7:
			settings.spreading_factor = SF_8;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_7;
			break;
		case 8:
			settings.spreading_factor = SF_8;
			settings.band_width = BW_500_KHz;
			settings.coding_rate = CR_4_8;
			break;
	}
	sx127x_init(&sx1278_phy, &settings);
}

uint32_t sx127x_spi_transmit(uint8_t* buffer, uint32_t size)
{
    HAL_GPIO_WritePin(NSS_LoRa_GPIO_Port, NSS_LoRa_Pin, GPIO_PIN_RESET);
	
    HAL_SPI_Transmit(&hspi2, buffer, size, 1000);

    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
        ;

    HAL_GPIO_WritePin(NSS_LoRa_GPIO_Port, NSS_LoRa_Pin, GPIO_PIN_SET);

    return 0;
}

uint32_t sx127x_spi_transmit_receive(uint8_t* tx_buffer, uint32_t tx_size, uint8_t* rx_buffer, uint32_t rx_size)
{
    HAL_GPIO_WritePin(NSS_LoRa_GPIO_Port, NSS_LoRa_Pin, GPIO_PIN_RESET);

//		HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, rx_size, 0x1000);
		
    HAL_SPI_Transmit(&hspi2, tx_buffer, tx_size, 1000);

    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
        ;

    HAL_SPI_Receive(&hspi2, rx_buffer, rx_size, 1000);

    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
        ;

    HAL_GPIO_WritePin(NSS_LoRa_GPIO_Port, NSS_LoRa_Pin, GPIO_PIN_SET);

    return 0;
}

void sx127x_delay(uint32_t delay)
{
    HAL_Delay(delay);
}

void sx127x_reset_control(bool state)
{
    HAL_GPIO_WritePin(RESET_LoRa_GPIO_Port, RESET_LoRa_Pin,
        (state) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void sx127x_rx_callback(uint8_t* buffer, uint8_t size)
{
    printf("RX DONE. DATA: %s | RSSI: %d | SNR: %d\r\n", buffer, sx127x_get_last_packet_rssi(&sx1278_phy), sx127x_get_last_packet_snr(&sx1278_phy));
}

void sx127x_tx_callback()
{
    printf("TX DONE\r\n");
}

void sx127x_rx_timeout_callback()
{
    printf("RX TIMEOUT\r\n");
}

void sx127x_rx_crc_error_callback()
{
    printf("RX CRC ERROR\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    switch (GPIO_Pin) {
//    case DIO0_Pin:
//        sx127x_dio_0_callback(sx1278_phy);
//        break;
//    case DIO1_Pin:
//        sx127x_dio_1_callback(sx1278_phy);
//        break;
//    case DIO3_Pin:
//        sx127x_dio_3_callback(sx1278_phy);
//        break;
//    default:
//        break;
//    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
