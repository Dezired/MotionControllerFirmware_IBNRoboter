#include "inbetriebnahme.h"
#include "main.h"

float angle = 0;
int16_t step = 0;

void IBN_Init()
{
	//  activate LEDs (test)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(2000);
	// deactivate LEDs (test)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

// GPIO Initialisierung
void IBN_GPIO_Init(void) {
	// Driver A
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();  // Timer 1 aktivieren

	GPIO_InitTypeDef GPIO_InitStructA = {0};

	// PC0-PC3 als Alternativfunktion (PWM) konfigurieren
	GPIO_InitStructA.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStructA.Mode = GPIO_MODE_AF_PP;  // Alternativfunktion für PWM
	GPIO_InitStructA.Pull = GPIO_NOPULL;
	GPIO_InitStructA.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructA.Alternate = GPIO_AF2_TIM1;  // Timer 1 zuordnen
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructA);

	// Timer 2 für PWM konfigurieren
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 64 - 1;  // Teilt den Takt für eine PWM-Frequenz um ~1 kHz
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;  // 1000 Schritte für 100% Duty Cycle
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim1);

	TIM_OC_InitTypeDef sConfigOCA = {0};
	sConfigOCA.OCMode = TIM_OCMODE_PWM1;
	sConfigOCA.Pulse = 0;
	sConfigOCA.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOCA.OCFastMode = TIM_OCFAST_DISABLE;

	// PWM-Kanäle für PC0-PC3 konfigurieren
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCA, TIM_CHANNEL_1);  // PC0
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCA, TIM_CHANNEL_2);  // PC1
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCA, TIM_CHANNEL_3);  // PC2
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCA, TIM_CHANNEL_4);  // PC3

	// PWM starten (HW-PWM aktiv)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PC0
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PC1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // PC2
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PC3

	// Driver B
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_TIM20_CLK_ENABLE();  // Timer 20 aktivieren

	GPIO_InitTypeDef GPIO_InitStructB = {0};

	// PF12-PF15 als Alternativfunktion (PWM) konfigurieren
	GPIO_InitStructB.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructB.Mode = GPIO_MODE_AF_PP;  // Alternativfunktion für PWM
	GPIO_InitStructB.Pull = GPIO_NOPULL;
	GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructB.Alternate = GPIO_AF2_TIM20;  // Timer 20 zuordnen
	HAL_GPIO_Init(GPIOF, &GPIO_InitStructB);

	// Timer 20 für PWM konfigurieren
	htim20.Instance = TIM20;
	htim20.Init.Prescaler = 64 - 1;  // Teilt den Takt für eine PWM-Frequenz um ~1 kHz
	htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim20.Init.Period = 1000 - 1;  // 1000 Schritte für 100% Duty Cycle
	htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim20);

	TIM_OC_InitTypeDef sConfigOCB = {0};
	sConfigOCB.OCMode = TIM_OCMODE_PWM1;
	sConfigOCB.Pulse = 0;
	sConfigOCB.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOCB.OCFastMode = TIM_OCFAST_DISABLE;

	// PWM-Kanäle für PC0-PC3 konfigurieren
	HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOCB, TIM_CHANNEL_1);  // PF12
	HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOCB, TIM_CHANNEL_2);  // PF13
	HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOCB, TIM_CHANNEL_3);  // PF14
	HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOCB, TIM_CHANNEL_4);  // PF15

	// PWM starten (HW-PWM aktiv)
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);  // PF12
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);  // PF13
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3);  // PF14
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_4);  // PF15
}

void IBN_Stepper_Init()
{
    IBN_GPIO_Init();       // GPIOs initialisieren
    IBN_Set_Mode();        // Modus konfigurieren
    IBN_Enable_Driver('A');   // Treiber aktivieren
}

// Setze den Betriebsmodus des DRV8412
void IBN_Set_Mode(void) {
	// Driver A
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);  // Mode1 LOW
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);  // Mode2 LOW
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);  // Mode3 LOW

	//Driver B
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);  // Mode1 LOW
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);  // Mode2 LOW
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);  // Mode3 LOW
}

// Setze den DRV8412 in den aktiven Zustand
void IBN_Enable_Driver(char driver) {
	// Driver A
	if(driver == 'A')
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // Reset HIGH, Treiber aktiv
	else if(driver == 'B')
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(10);  // Kurze Wartezeit
}

// Überprüfe auf Überhitzung oder Fehler
uint8_t IBN_Check_Errors(void) {
	// Driver A
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
        return 1;  // Überhitzung
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {
        return 2;  // Fehler
    }

	//Driver B
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {
	        return 11;  // Überhitzung
	    }
	    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4) == GPIO_PIN_RESET) {
	        return 12;  // Fehler
	    }
    return 0;  // Alles in Ordnung
}

// Schrittmotor-Test (Vollschritt)
void IBN_Step(float targetAngle, char driver) {
	int16_t targetStep = (int16_t) (targetAngle / 0.9);

	if(targetStep > step)
	{
		IBN_Step_Motor_Forward(driver);
	}
	else if (targetStep < step)
	{
		IBN_Step_Motor_Backward(driver);
	}
	else
	{

	}
}



void IBN_Set_PWM_DutyCycle(uint32_t channel, uint16_t duty, char driver) {
	// Driver A
	if(driver == 'A')
		__HAL_TIM_SET_COMPARE(&htim1, channel, duty);  // Duty zwischen 0 und 1000 setzen
	// Driver B
	else if(driver == 'B')
		__HAL_TIM_SET_COMPARE(&htim20, channel, duty);
}

void IBN_Step_Motor_Forward(char driver) {

    uint8_t pstep = (step + 1) % 8;  // Schritt erhöhen, aber innerhalb 0-7 bleiben
    step = step + 1;

    switch (pstep) {
        case 0:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 1:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 2:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 3:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 4:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 5:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
        case 6:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
        case 7:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
    }
}

void IBN_Step_Motor_Backward(char driver) {
	uint8_t pstep = ((step - 1) % 8 + 8) % 8;  // Schritt verringern, aber innerhalb 0-7 bleiben
    step = step - 1;

    switch (pstep) {
        case 0:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 1:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 2:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 3:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 4:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 0, driver);
            break;
        case 5:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
        case 6:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
        case 7:
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_1, 170, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_2, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_3, 0, driver);
            IBN_Set_PWM_DutyCycle(TIM_CHANNEL_4, 170, driver);
            break;
    }
}
