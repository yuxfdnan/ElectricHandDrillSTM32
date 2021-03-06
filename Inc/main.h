/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define BLDC_PWM_Freq 25000

#define LED_Ctrl_Pin GPIO_PIN_0
#define LED_Ctrl_GPIO_Port GPIOA
#define MidAD_Pin GPIO_PIN_1
#define MidAD_GPIO_Port GPIOA
#define CurrentAD_Pin GPIO_PIN_2
#define CurrentAD_GPIO_Port GPIOA
#define SwtichAD_Pin GPIO_PIN_3
#define SwtichAD_GPIO_Port GPIOA
#define MotorAD_A_Pin GPIO_PIN_4
#define MotorAD_A_GPIO_Port GPIOA
#define MotorAD_B_Pin GPIO_PIN_5
#define MotorAD_B_GPIO_Port GPIOA
#define MotorAD_C_Pin GPIO_PIN_6
#define MotorAD_C_GPIO_Port GPIOA
#define Low3_Pin GPIO_PIN_7
#define Low3_GPIO_Port GPIOA
#define Low2_Pin GPIO_PIN_0
#define Low2_GPIO_Port GPIOB
#define Low1_Pin GPIO_PIN_1
#define Low1_GPIO_Port GPIOB
#define High3_Pin GPIO_PIN_8
#define High3_GPIO_Port GPIOA
#define High2_Pin GPIO_PIN_9
#define High2_GPIO_Port GPIOA
#define High1_Pin GPIO_PIN_10
#define High1_GPIO_Port GPIOA
#define Swtich_Pin GPIO_PIN_11
#define Swtich_GPIO_Port GPIOA
#define SwtichIn_Pin GPIO_PIN_12
#define SwtichIn_GPIO_Port GPIOA
#define Led1_Pin GPIO_PIN_3
#define Led1_GPIO_Port GPIOB
#define Led2_Pin GPIO_PIN_4
#define Led2_GPIO_Port GPIOB
#define Led3_Pin GPIO_PIN_5
#define Led3_GPIO_Port GPIOB
#define Key1_Pin GPIO_PIN_6
#define Key1_GPIO_Port GPIOB
#define Key2_Pin GPIO_PIN_7
#define Key2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/** @brief Reset TIM IC polarity
  * @param  __HANDLE__ TIM handle
  * @param  __CHANNEL__ specifies TIM Channel
  * @param  __POLARITY__
  *           This parameter can be one of the following values:
  *            @arg TIM_CCER_CC1P:
  *            @arg TIM_CCER_CC2P:
  *            @arg TIM_CCER_CC3P:
  *            @arg TIM_CCER_CC4P:
  *            @arg TIM_CCER_CC1NP:
  *            @arg TIM_CCER_CC2NP:
  *            @arg TIM_CCER_CC3NP:
  *            @arg TIM_CCER_CC4NP:
  * @retval None
  */
//TODO ������
#define TIM_RESET_OC_POLARITY(__HANDLE__, __CHANNEL__, __POLARITY__) \
 ((__HANDLE__)->Instance->CCER &= (uint16_t)~(__POLARITY__))

/**
  * @brief  Sets the TIM Complementary Output Compare Polarity
  * @param  __HANDLE__ TIM handle.
  * @param  __CHANNEL__ TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  __POLARITY__ Polarity for TIx source
  *            @arg TIM_OCPOLARITY_HIGH: active high
  *            @arg TIM_OCPOLARITY_LOW: active low
  *            @arg TIM_OCNPOLARITY_HIGH: active high
  *            @arg TIM_OCNPOLARITY_LOW: active low
  * @note  The polarity TIM_OCNPOLARITY_HIGH/TIM_OCNPOLARITY_LOW is not authorized  for TIM Channel 4.
  * @retval None
  */
#define __HAL_TIM_SET_OC_POLARITY(__HANDLE__, __CHANNEL__, __POLARITY__)    \
        do{                                                                     \
          TIM_RESET_OC_POLARITY((__HANDLE__), (__CHANNEL__), (__POLARITY__));     \
          TIM_SET_CAPTUREPOLARITY((__HANDLE__), (__CHANNEL__), (__POLARITY__)); \
        }while(0)
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
