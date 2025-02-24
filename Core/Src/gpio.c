/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOT_A_RESET_Pin|LED_GREEN_RUN_NEU_Pin|LED_YELLOW_WARNING_NEU_Pin|MOT_B_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOT_A_M1_Pin|MOT_A_M2_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_ERROR_NEU_Pin|LED_GREEN_ON_TARGET_NEU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SIN_COS_A_RESET_NEU_Pin|SIN_COS_B_RESET_NEU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MOT_B_M3_Pin|MOT_B_M2_Pin|MOT_B_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOT_A_RESET_Pin LED_GREEN_RUN_NEU_Pin LED_YELLOW_WARNING_NEU_Pin MOT_B_RESET_Pin */
  GPIO_InitStruct.Pin = MOT_A_RESET_Pin|LED_GREEN_RUN_NEU_Pin|LED_YELLOW_WARNING_NEU_Pin|MOT_B_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 MOT_A_FAULT_Pin MOT_B_TEMP_FAULT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|MOT_A_FAULT_Pin|MOT_B_TEMP_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_A_M1_Pin MOT_A_M2_Pin PF5 */
  GPIO_InitStruct.Pin = MOT_A_M1_Pin|MOT_A_M2_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_ERROR_NEU_Pin LED_GREEN_ON_TARGET_NEU_Pin */
  GPIO_InitStruct.Pin = LED_RED_ERROR_NEU_Pin|LED_GREEN_ON_TARGET_NEU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SIN_COS_A_RESET_NEU_Pin SIN_COS_B_RESET_NEU_Pin */
  GPIO_InitStruct.Pin = SIN_COS_A_RESET_NEU_Pin|SIN_COS_B_RESET_NEU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SIN_COS_A_FAULT_NEU_Pin SIN_COS_B_FAULT_NEU_Pin */
  GPIO_InitStruct.Pin = SIN_COS_A_FAULT_NEU_Pin|SIN_COS_B_FAULT_NEU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_B_M3_Pin MOT_B_M2_Pin MOT_B_M1_Pin */
  GPIO_InitStruct.Pin = MOT_B_M3_Pin|MOT_B_M2_Pin|MOT_B_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MOT_B_FAULT_Pin */
  GPIO_InitStruct.Pin = MOT_B_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOT_B_FAULT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
