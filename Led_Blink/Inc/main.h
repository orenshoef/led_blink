/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RF_SAI1_MCLK_A_Pin GPIO_PIN_2
#define RF_SAI1_MCLK_A_GPIO_Port GPIOE
#define RF_SAI1_SD_B_Pin GPIO_PIN_3
#define RF_SAI1_SD_B_GPIO_Port GPIOE
#define RF_SAI1_FS_A_Pin GPIO_PIN_4
#define RF_SAI1_FS_A_GPIO_Port GPIOE
#define RF_SAI1_SCK_A_Pin GPIO_PIN_5
#define RF_SAI1_SCK_A_GPIO_Port GPIOE
#define RF_SAI1_SD_A_Pin GPIO_PIN_6
#define RF_SAI1_SD_A_GPIO_Port GPIOE
#define TAMPER_WKUP_KEY_Pin GPIO_PIN_13
#define TAMPER_WKUP_KEY_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define FMC_A0_Pin GPIO_PIN_0
#define FMC_A0_GPIO_Port GPIOF
#define FMC_A1_Pin GPIO_PIN_1
#define FMC_A1_GPIO_Port GPIOF
#define FMC_A2_Pin GPIO_PIN_2
#define FMC_A2_GPIO_Port GPIOF
#define FMC_A3_Pin GPIO_PIN_3
#define FMC_A3_GPIO_Port GPIOF
#define FMC_A4_Pin GPIO_PIN_4
#define FMC_A4_GPIO_Port GPIOF
#define FMC_A5_Pin GPIO_PIN_5
#define FMC_A5_GPIO_Port GPIOF
#define QSPI_BK1_IO3_Pin GPIO_PIN_6
#define QSPI_BK1_IO3_GPIO_Port GPIOF
#define QSPI_BK1_IO2_Pin GPIO_PIN_7
#define QSPI_BK1_IO2_GPIO_Port GPIOF
#define QSPI_BK1_IO0_Pin GPIO_PIN_8
#define QSPI_BK1_IO0_GPIO_Port GPIOF
#define QSPI_BK1_IO1_Pin GPIO_PIN_9
#define QSPI_BK1_IO1_GPIO_Port GPIOF
#define RF_SPI2_CS_Pin GPIO_PIN_10
#define RF_SPI2_CS_GPIO_Port GPIOF
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define ULPI_STP_Pin GPIO_PIN_0
#define ULPI_STP_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_1
#define I2S3_SD_GPIO_Port GPIOC
#define ULPI_DIR_Pin GPIO_PIN_2
#define ULPI_DIR_GPIO_Port GPIOC
#define ULPI_NXT_Pin GPIO_PIN_3
#define ULPI_NXT_GPIO_Port GPIOC
#define MFX_IRQ_OUT_Pin GPIO_PIN_0
#define MFX_IRQ_OUT_GPIO_Port GPIOA
#define SAI2_MCLKB_Pin GPIO_PIN_1
#define SAI2_MCLKB_GPIO_Port GPIOA
#define SAI2_SCKB_Pin GPIO_PIN_2
#define SAI2_SCKB_GPIO_Port GPIOA
#define ULPI_D0_Pin GPIO_PIN_3
#define ULPI_D0_GPIO_Port GPIOA
#define ADC12_IN4_Pin GPIO_PIN_4
#define ADC12_IN4_GPIO_Port GPIOA
#define ULPI_CK_Pin GPIO_PIN_5
#define ULPI_CK_GPIO_Port GPIOA
#define FMC_SDNWE_Pin GPIO_PIN_7
#define FMC_SDNWE_GPIO_Port GPIOA
#define FMC_SDNE0_Pin GPIO_PIN_4
#define FMC_SDNE0_GPIO_Port GPIOC
#define FMC_SDCKE0_Pin GPIO_PIN_5
#define FMC_SDCKE0_GPIO_Port GPIOC
#define ULPI_D1_Pin GPIO_PIN_0
#define ULPI_D1_GPIO_Port GPIOB
#define ULPI_D2_Pin GPIO_PIN_1
#define ULPI_D2_GPIO_Port GPIOB
#define ULPI_D4_Pin GPIO_PIN_2
#define ULPI_D4_GPIO_Port GPIOB
#define FMC_SDNRAS_Pin GPIO_PIN_11
#define FMC_SDNRAS_GPIO_Port GPIOF
#define FMC_A6_Pin GPIO_PIN_12
#define FMC_A6_GPIO_Port GPIOF
#define FMC_A7_Pin GPIO_PIN_13
#define FMC_A7_GPIO_Port GPIOF
#define FMC_A8_Pin GPIO_PIN_14
#define FMC_A8_GPIO_Port GPIOF
#define FMC_A9_Pin GPIO_PIN_15
#define FMC_A9_GPIO_Port GPIOF
#define FMC_A10__MT48LC4M16A2P_6A_A10_Pin GPIO_PIN_0
#define FMC_A10__MT48LC4M16A2P_6A_A10_GPIO_Port GPIOG
#define FMC_A11_Pin GPIO_PIN_1
#define FMC_A11_GPIO_Port GPIOG
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define ULPI_D3_Pin GPIO_PIN_10
#define ULPI_D3_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_11
#define LED_GREEN_GPIO_Port GPIOB
#define ULPI_D5_Pin GPIO_PIN_12
#define ULPI_D5_GPIO_Port GPIOB
#define ULPI_D6_Pin GPIO_PIN_13
#define ULPI_D6_GPIO_Port GPIOB
#define RF_SPI2_MISO_Pin GPIO_PIN_14
#define RF_SPI2_MISO_GPIO_Port GPIOB
#define RF_SPI2_MOSI_2_SD_Pin GPIO_PIN_15
#define RF_SPI2_MOSI_2_SD_GPIO_Port GPIOB
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define SAI2_SDA_Pin GPIO_PIN_11
#define SAI2_SDA_GPIO_Port GPIOD
#define I2C4_SCL_Pin GPIO_PIN_12
#define I2C4_SCL_GPIO_Port GPIOD
#define I2C4_SDA_Pin GPIO_PIN_13
#define I2C4_SDA_GPIO_Port GPIOD
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define FMC_A12_Pin GPIO_PIN_2
#define FMC_A12_GPIO_Port GPIOG
#define MFX_WAKEUP_Pin GPIO_PIN_3
#define MFX_WAKEUP_GPIO_Port GPIOG
#define FMC_BA0_Pin GPIO_PIN_4
#define FMC_BA0_GPIO_Port GPIOG
#define FMC_BA1_Pin GPIO_PIN_5
#define FMC_BA1_GPIO_Port GPIOG
#define QSPI_BK1_NCS_Pin GPIO_PIN_6
#define QSPI_BK1_NCS_GPIO_Port GPIOG
#define RF_USART6_CK_Pin GPIO_PIN_7
#define RF_USART6_CK_GPIO_Port GPIOG
#define FMC_SDCLK_Pin GPIO_PIN_8
#define FMC_SDCLK_GPIO_Port GPIOG
#define RF_USART6_RX_Pin GPIO_PIN_7
#define RF_USART6_RX_GPIO_Port GPIOC
#define SDCARD_D0_Pin GPIO_PIN_8
#define SDCARD_D0_GPIO_Port GPIOC
#define SDCARD_D1_Pin GPIO_PIN_9
#define SDCARD_D1_GPIO_Port GPIOC
#define RF_INT1_Pin GPIO_PIN_8
#define RF_INT1_GPIO_Port GPIOA
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define JTMS_SWDIO_Pin GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port GPIOA
#define JTCK_SWCLK_Pin GPIO_PIN_14
#define JTCK_SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SDCARD_D2_Pin GPIO_PIN_10
#define SDCARD_D2_GPIO_Port GPIOC
#define SDCARD_D3_Pin GPIO_PIN_11
#define SDCARD_D3_GPIO_Port GPIOC
#define SDCARD_CK_Pin GPIO_PIN_12
#define SDCARD_CK_GPIO_Port GPIOC
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define SDCARD_CMD_Pin GPIO_PIN_2
#define SDCARD_CMD_GPIO_Port GPIOD
#define QSPI_CLK_Pin GPIO_PIN_3
#define QSPI_CLK_GPIO_Port GPIOD
#define FMC_NOE_Pin GPIO_PIN_4
#define FMC_NOE_GPIO_Port GPIOD
#define FMC_NWE_Pin GPIO_PIN_5
#define FMC_NWE_GPIO_Port GPIOD
#define RF_INT2_Pin GPIO_PIN_6
#define RF_INT2_GPIO_Port GPIOD
#define FMC_NE1_Pin GPIO_PIN_7
#define FMC_NE1_GPIO_Port GPIOD
#define SAI2_FSB_Pin GPIO_PIN_9
#define SAI2_FSB_GPIO_Port GPIOG
#define SAI2_SDB_Pin GPIO_PIN_10
#define SAI2_SDB_GPIO_Port GPIOG
#define SPDIF_RX0_Pin GPIO_PIN_11
#define SPDIF_RX0_GPIO_Port GPIOG
#define RF_USART6_RTS_Pin GPIO_PIN_12
#define RF_USART6_RTS_GPIO_Port GPIOG
#define RF_USART6_CTS_Pin GPIO_PIN_13
#define RF_USART6_CTS_GPIO_Port GPIOG
#define RF_USART6_TX_Pin GPIO_PIN_14
#define RF_USART6_TX_GPIO_Port GPIOG
#define FMC_SDNCAS_Pin GPIO_PIN_15
#define FMC_SDNCAS_GPIO_Port GPIOG
#define I2S3_CK_Pin GPIO_PIN_3
#define I2S3_CK_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOB
#define ULPI_D7_Pin GPIO_PIN_5
#define ULPI_D7_GPIO_Port GPIOB
#define MIC_TIM4_CH1_Pin GPIO_PIN_6
#define MIC_TIM4_CH1_GPIO_Port GPIOB
#define MIC_TIM4_CH2_Pin GPIO_PIN_7
#define MIC_TIM4_CH2_GPIO_Port GPIOB
#define RF_TIM2_CH1_Pin GPIO_PIN_8
#define RF_TIM2_CH1_GPIO_Port GPIOB
#define RF_TIM2_CH2_Pin GPIO_PIN_9
#define RF_TIM2_CH2_GPIO_Port GPIOB
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
