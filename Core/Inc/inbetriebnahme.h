#ifndef INBETRIEBNAHME_H
#define INBETRIEBNAHME_H
#include "main.h"
#include <stdlib.h>
#include "stm32g4xx_hal.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim20;
extern float angle;
extern int16_t step;

void IBN_Init();
void IBN_Stepper_Init();
void IBN_GPIO_Init();
void IBN_Set_Mode();
void IBN_Enable_Driver(char driver);
uint8_t IBN_Check_Errors(void);
void IBN_Step(float targetAngle, char driver);
void IBN_Set_PWM_DutyCycle(uint32_t channel, uint16_t duty, char driver);
void IBN_Step_Motor_Forward(char driver);
void IBN_Step_Motor_Backward(char driver);
#endif /* INBETRIEBNAHME_H */
