/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins
     PE2   ------> SAI1_MCLK_A
     PE3   ------> SAI1_SD_B
     PE4   ------> SAI1_FS_A
     PE5   ------> SAI1_SCK_A
     PE6   ------> SAI1_SD_A
     PF0   ------> FMC_A0
     PF1   ------> FMC_A1
     PF2   ------> FMC_A2
     PF3   ------> FMC_A3
     PF4   ------> FMC_A4
     PF5   ------> FMC_A5
     PF6   ------> QUADSPI_BK1_IO3
     PF7   ------> QUADSPI_BK1_IO2
     PF8   ------> QUADSPI_BK1_IO0
     PF9   ------> QUADSPI_BK1_IO1
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC1   ------> I2S3_SD
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PC3   ------> USB_OTG_HS_ULPI_NXT
     PA1   ------> SAI2_MCLK_B
     PA2   ------> SAI2_SCK_B
     PA3   ------> USB_OTG_HS_ULPI_D0
     PA4   ------> ADCx_IN4
     PA5   ------> USB_OTG_HS_ULPI_CK
     PA7   ------> FMC_SDNWE
     PC4   ------> FMC_SDNE0
     PC5   ------> FMC_SDCKE0
     PB0   ------> USB_OTG_HS_ULPI_D1
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB2   ------> USB_OTG_HS_ULPI_D4
     PF11   ------> FMC_SDNRAS
     PF12   ------> FMC_A6
     PF13   ------> FMC_A7
     PF14   ------> FMC_A8
     PF15   ------> FMC_A9
     PG0   ------> FMC_A10
     PG1   ------> FMC_A11
     PE7   ------> FMC_D4_DA4
     PE8   ------> FMC_D5_DA5
     PE9   ------> FMC_D6_DA6
     PE10   ------> FMC_D7_DA7
     PE11   ------> FMC_D8_DA8
     PE12   ------> FMC_D9_DA9
     PE13   ------> FMC_D10_DA10
     PE14   ------> FMC_D11_DA11
     PE15   ------> FMC_D12_DA12
     PB10   ------> USB_OTG_HS_ULPI_D3
     PB12   ------> USB_OTG_HS_ULPI_D5
     PB13   ------> USB_OTG_HS_ULPI_D6
     PB14   ------> SPI2_MISO
     PB15   ------> I2S2_SD
     PD8   ------> FMC_D13_DA13
     PD9   ------> FMC_D14_DA14
     PD10   ------> FMC_D15_DA15
     PD11   ------> SAI2_SD_A
     PD12   ------> FMPI2C1_SCL
     PD13   ------> FMPI2C1_SDA
     PD14   ------> FMC_D0_DA0
     PD15   ------> FMC_D1_DA1
     PG2   ------> FMC_A12
     PG4   ------> FMC_A14_BA0
     PG5   ------> FMC_A15_BA1
     PG6   ------> QUADSPI_BK1_NCS
     PG7   ------> USART6_CK
     PG8   ------> FMC_SDCLK
     PC7   ------> USART6_RX
     PC8   ------> SDIO_D0
     PC9   ------> SDIO_D1
     PA9   ------> USART1_TX
     PA10   ------> USART1_RX
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> SDIO_D2
     PC11   ------> SDIO_D3
     PC12   ------> SDIO_CK
     PD0   ------> FMC_D2_DA2
     PD1   ------> FMC_D3_DA3
     PD2   ------> SDIO_CMD
     PD3   ------> QUADSPI_CLK
     PD4   ------> FMC_NOE
     PD5   ------> FMC_NWE
     PD7   ------> FMC_NE1
     PG9   ------> SAI2_FS_B
     PG10   ------> SAI2_SD_B
     PG11   ------> SPDIFRX_IN0
     PG12   ------> USART6_RTS
     PG13   ------> USART6_CTS
     PG14   ------> USART6_TX
     PG15   ------> FMC_SDNCAS
     PB3   ------> I2S3_CK
     PB5   ------> USB_OTG_HS_ULPI_D7
     PB6   ------> S_TIM4_CH1
     PB7   ------> S_TIM4_CH2
     PB8   ------> S_TIM2_CH1_ETR
     PB9   ------> S_TIM2_CH2
     PE0   ------> FMC_NBL0
     PE1   ------> FMC_NBL1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : RF_SAI1_MCLK_A_Pin RF_SAI1_SD_B_Pin RF_SAI1_FS_A_Pin RF_SAI1_SCK_A_Pin 
                           RF_SAI1_SD_A_Pin */
  GPIO_InitStruct.Pin = RF_SAI1_MCLK_A_Pin|RF_SAI1_SD_B_Pin|RF_SAI1_FS_A_Pin|RF_SAI1_SCK_A_Pin 
                          |RF_SAI1_SD_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TAMPER_WKUP_KEY_Pin */
  GPIO_InitStruct.Pin = TAMPER_WKUP_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TAMPER_WKUP_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_A0_Pin FMC_A1_Pin FMC_A2_Pin FMC_A3_Pin 
                           FMC_A4_Pin FMC_A5_Pin FMC_SDNRAS_Pin FMC_A6_Pin 
                           FMC_A7_Pin FMC_A8_Pin FMC_A9_Pin */
  GPIO_InitStruct.Pin = FMC_A0_Pin|FMC_A1_Pin|FMC_A2_Pin|FMC_A3_Pin 
                          |FMC_A4_Pin|FMC_A5_Pin|FMC_SDNRAS_Pin|FMC_A6_Pin 
                          |FMC_A7_Pin|FMC_A8_Pin|FMC_A9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO3_Pin QSPI_BK1_IO2_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO3_Pin|QSPI_BK1_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO0_Pin QSPI_BK1_IO1_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO0_Pin|QSPI_BK1_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_SPI2_CS_Pin */
  GPIO_InitStruct.Pin = RF_SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin|ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI3;
  HAL_GPIO_Init(I2S3_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_MCLKB_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_MCLKB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SCKB_Pin */
  GPIO_InitStruct.Pin = SAI2_SCKB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
  HAL_GPIO_Init(SAI2_SCKB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D0_Pin ULPI_CK_Pin */
  GPIO_InitStruct.Pin = ULPI_D0_Pin|ULPI_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC12_IN4_Pin */
  GPIO_InitStruct.Pin = ADC12_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC12_IN4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FMC_SDNWE_Pin */
  GPIO_InitStruct.Pin = FMC_SDNWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(FMC_SDNWE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNE0_Pin FMC_SDCKE0_Pin */
  GPIO_InitStruct.Pin = FMC_SDNE0_Pin|FMC_SDCKE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D1_Pin ULPI_D2_Pin ULPI_D4_Pin ULPI_D3_Pin 
                           ULPI_D5_Pin ULPI_D6_Pin ULPI_D7_Pin */
  GPIO_InitStruct.Pin = ULPI_D1_Pin|ULPI_D2_Pin|ULPI_D4_Pin|ULPI_D3_Pin 
                          |ULPI_D5_Pin|ULPI_D6_Pin|ULPI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_A10__MT48LC4M16A2P_6A_A10_Pin FMC_A11_Pin FMC_A12_Pin FMC_BA0_Pin 
                           FMC_BA1_Pin FMC_SDCLK_Pin FMC_SDNCAS_Pin */
  GPIO_InitStruct.Pin = FMC_A10__MT48LC4M16A2P_6A_A10_Pin|FMC_A11_Pin|FMC_A12_Pin|FMC_BA0_Pin 
                          |FMC_BA1_Pin|FMC_SDCLK_Pin|FMC_SDNCAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D4_Pin FMC_D5_Pin FMC_D6_Pin FMC_D7_Pin 
                           FMC_D8_Pin FMC_D9_Pin FMC_D10_Pin FMC_D11_Pin 
                           FMC_D12_Pin FMC_NBL0_Pin FMC_NBL1_Pin */
  GPIO_InitStruct.Pin = FMC_D4_Pin|FMC_D5_Pin|FMC_D6_Pin|FMC_D7_Pin 
                          |FMC_D8_Pin|FMC_D9_Pin|FMC_D10_Pin|FMC_D11_Pin 
                          |FMC_D12_Pin|FMC_NBL0_Pin|FMC_NBL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_SPI2_MISO_Pin */
  GPIO_InitStruct.Pin = RF_SPI2_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(RF_SPI2_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_SPI2_MOSI_2_SD_Pin */
  GPIO_InitStruct.Pin = RF_SPI2_MOSI_2_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(RF_SPI2_MOSI_2_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D13_Pin FMC_D14_Pin FMC_D15_Pin FMC_D0_Pin 
                           FMC_D1_Pin FMC_D2_Pin FMC_D3_Pin FMC_NOE_Pin 
                           FMC_NWE_Pin FMC_NE1_Pin */
  GPIO_InitStruct.Pin = FMC_D13_Pin|FMC_D14_Pin|FMC_D15_Pin|FMC_D0_Pin 
                          |FMC_D1_Pin|FMC_D2_Pin|FMC_D3_Pin|FMC_NOE_Pin 
                          |FMC_NWE_Pin|FMC_NE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C4_SCL_Pin I2C4_SDA_Pin */
  GPIO_InitStruct.Pin = I2C4_SCL_Pin|I2C4_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_FMPI2C1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MFX_WAKEUP_Pin */
  GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_BK1_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(QSPI_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_USART6_CK_Pin RF_USART6_RTS_Pin RF_USART6_CTS_Pin RF_USART6_TX_Pin */
  GPIO_InitStruct.Pin = RF_USART6_CK_Pin|RF_USART6_RTS_Pin|RF_USART6_CTS_Pin|RF_USART6_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_USART6_RX_Pin */
  GPIO_InitStruct.Pin = RF_USART6_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(RF_USART6_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCARD_D0_Pin SDCARD_D1_Pin SDCARD_D2_Pin SDCARD_D3_Pin 
                           SDCARD_CK_Pin */
  GPIO_InitStruct.Pin = SDCARD_D0_Pin|SDCARD_D1_Pin|SDCARD_D2_Pin|SDCARD_D3_Pin 
                          |SDCARD_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_INT1_Pin */
  GPIO_InitStruct.Pin = RF_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART1_TX_Pin USART1_RX_Pin */
  GPIO_InitStruct.Pin = USART1_TX_Pin|USART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_DM_Pin USB_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_FS_DM_Pin|USB_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SDCARD_CMD_Pin */
  GPIO_InitStruct.Pin = SDCARD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(SDCARD_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_CLK_Pin */
  GPIO_InitStruct.Pin = QSPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(QSPI_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_INT2_Pin */
  GPIO_InitStruct.Pin = RF_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_FSB_Pin SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_FSB_Pin|SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_CK_Pin */
  GPIO_InitStruct.Pin = I2S3_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MIC_TIM4_CH1_Pin MIC_TIM4_CH2_Pin */
  GPIO_InitStruct.Pin = MIC_TIM4_CH1_Pin|MIC_TIM4_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_TIM2_CH1_Pin RF_TIM2_CH2_Pin */
  GPIO_InitStruct.Pin = RF_TIM2_CH1_Pin|RF_TIM2_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_SPI2_CS_GPIO_Port, RF_SPI2_CS_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
