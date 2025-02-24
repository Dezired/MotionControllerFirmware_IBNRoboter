/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for IBNblinkLED */
osThreadId_t IBNblinkLEDHandle;
const osThreadAttr_t IBNblinkLED_attributes = {
  .name = "IBNblinkLED",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for IBNstepper */
osThreadId_t IBNstepperHandle;
const osThreadAttr_t IBNstepper_attributes = {
  .name = "IBNstepper",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for IBNerrorHandler */
osThreadId_t IBNerrorHandlerHandle;
const osThreadAttr_t IBNerrorHandler_attributes = {
  .name = "IBNerrorHandler",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void startIBNBlinkLED(void *argument);
void startIBNStepper(void *argument);
void startIBNerrorHandler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IBNblinkLED */
  IBNblinkLEDHandle = osThreadNew(startIBNBlinkLED, NULL, &IBNblinkLED_attributes);

  /* creation of IBNstepper */
  IBNstepperHandle = osThreadNew(startIBNStepper, NULL, &IBNstepper_attributes);

  /* creation of IBNerrorHandler */
  IBNerrorHandlerHandle = osThreadNew(startIBNerrorHandler, NULL, &IBNerrorHandler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_startIBNBlinkLED */
/**
* @brief Function implementing the IBNblinkLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startIBNBlinkLED */
void startIBNBlinkLED(void *argument)
{
  /* USER CODE BEGIN startIBNBlinkLED */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    osDelay(500);
  }
  /* USER CODE END startIBNBlinkLED */
}

/* USER CODE BEGIN Header_startIBNStepper */
/**
* @brief Function implementing the IBNstepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startIBNStepper */
void startIBNStepper(void *argument)
{
  /* USER CODE BEGIN startIBNStepper */
  /* Infinite loop */

  for(;;)
  {
	  /*uint8_t error = IBN_Check_Errors();  // Prüfe Überhitzung/Fehler
	  if (error == 0) {
		  IBN_Step(720.0, 'A');  // Motor dreht sich in einer Schleife
		  osDelay(5);
	  } else if (error == 1) {
		  // Überhitzung erkannt
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // Treiber deaktivieren
		 // while (1);  // Stoppe das Programm
	  } else if (error == 2) {
		  // Fehler erkannt
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // Treiber deaktivieren
		 // while (1);  // Stoppe das Programm
	  }*/
	  volatile uint16_t encoder_value;
	  encoder_value = __HAL_TIM_GET_COUNTER(&htim2);

    osDelay(1);
  }
  /* USER CODE END startIBNStepper */
}

/* USER CODE BEGIN Header_startIBNerrorHandler */
/**
* @brief Function implementing the IBNerrorHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startIBNerrorHandler */
void startIBNerrorHandler(void *argument)
{
  /* USER CODE BEGIN startIBNerrorHandler */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t error = IBN_Check_Errors();
	  if ((error == 1)|(error == 2))
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  }
    osDelay(1000);
  }
  /* USER CODE END startIBNerrorHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

