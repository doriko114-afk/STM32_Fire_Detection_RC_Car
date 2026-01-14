/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main_with_patrol.c
 * @brief          : Main program body for RC Car with Patrol Mode.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------ */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Buzzer Pin (TIM1_CH1)
#define BUZZER_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_8

// Servo Pin (TIM2_CH1)
#define SERVO_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_15

// Flame Sensor Pin (Digital Out)
#define FLAME_SENSOR_GPIO_Port GPIOB
#define FLAME_SENSOR_Pin GPIO_PIN_12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// --- Patrol Mode Variables ---
// Use volatile as this can be changed by an interrupt (UART) and accessed in the main loop
volatile uint8_t patrol_mode_active = 0;

// Enum for patrol states
typedef enum {
	PATROL_STATE_STOPPED,
	PATROL_STATE_FORWARD_1,
	PATROL_STATE_TURN_1,
	PATROL_STATE_FORWARD_2,
	PATROL_STATE_TURN_2,
	PATROL_STATE_FORWARD_3,
	PATROL_STATE_TURN_3,
	PATROL_STATE_FORWARD_4,
	PATROL_STATE_TURN_4
} PatrolState;

PatrolState patrol_state = PATROL_STATE_STOPPED;
uint32_t patrol_state_timestamp = 0;

// Durations for patrol movements (in milliseconds)
const uint32_t PATROL_FORWARD_DURATION = 4000; // 2 seconds
const uint32_t PATROL_TURN_DURATION = 1700;  // 0.5 seconds
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void BUZZER_Start(void);
void BUZZER_Stop(void);
void check_flame_sensor_and_alert(void);
void activate_extinguisher_servo(void);
void patrol_handler(void); // New patrol handler function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

// --- Buzzer Control Functions ---
void BUZZER_Start(void) {
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void BUZZER_Stop(void) {
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}




// --- RC Car Control Functions (Cleaned up) ---
void smartcar_stop(void) {
	HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0);
	HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0);
	HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0);
	HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0);
	HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
	HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
	HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
	HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}

void smartcar_forward(void) {

	smartcar_stop();
	HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1);
	HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1);
	HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1);
	HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1);
}

void smartcar_backward(void) {
	smartcar_stop();
	HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1);
	HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1);
	HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1);
	HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1);
}

// Turn Right: Left wheels forward, Right wheels backward
void smartcar_right(void) {
	smartcar_stop();
	HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1);
	HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1);
	HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
	HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
}

// Turn Left: Right wheels forward, Left wheels backward
void smartcar_left(void) {

	smartcar_stop();
	HAL_GPIO_WritePin(LFF_GPIO_Port, LFB_Pin, 1);
	HAL_GPIO_WritePin(LBF_GPIO_Port, LBB_Pin, 1);
	HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
	HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}

// --- Patrol Mode Handler ---
void patrol_handler(void) {
	if (!patrol_mode_active) {
		return;
	}

	uint32_t current_time = HAL_GetTick();

	switch (patrol_state) {
	case PATROL_STATE_STOPPED:
		// Do nothing, waiting for 'p' command
		break;

	case PATROL_STATE_FORWARD_1:
		if (current_time - patrol_state_timestamp > PATROL_FORWARD_DURATION) {
			patrol_state = PATROL_STATE_TURN_1;
			patrol_state_timestamp = current_time;
			printf("Patrol: Turning 1\r\n");
			smartcar_right();
		}
		break;

	case PATROL_STATE_TURN_1:
		if (current_time - patrol_state_timestamp > PATROL_TURN_DURATION) {
			patrol_state = PATROL_STATE_FORWARD_2;
			patrol_state_timestamp = current_time;
			printf("Patrol: Forward 2\r\n");
			smartcar_forward();
		}
		break;

	case PATROL_STATE_FORWARD_2:
		if (current_time - patrol_state_timestamp > PATROL_FORWARD_DURATION) {
			patrol_state = PATROL_STATE_TURN_2;
			patrol_state_timestamp = current_time;
			printf("Patrol: Turning 2\r\n");
			smartcar_right();
		}
		break;

	case PATROL_STATE_TURN_2:
		if (current_time - patrol_state_timestamp > PATROL_TURN_DURATION) {
			patrol_state = PATROL_STATE_FORWARD_3;
			patrol_state_timestamp = current_time;
			printf("Patrol: Forward 3\r\n");
			smartcar_forward();
		}
		break;

	case PATROL_STATE_FORWARD_3:
		if (current_time - patrol_state_timestamp > PATROL_FORWARD_DURATION) {
			patrol_state = PATROL_STATE_TURN_3;
			patrol_state_timestamp = current_time;
			printf("Patrol: Turning 3\r\n");
			smartcar_right();
		}
		break;

	case PATROL_STATE_TURN_3:
		if (current_time - patrol_state_timestamp > PATROL_TURN_DURATION) {
			patrol_state = PATROL_STATE_FORWARD_4;
			patrol_state_timestamp = current_time;
			printf("Patrol: Forward 4\r\n");
			smartcar_forward();
		}
		break;

	case PATROL_STATE_FORWARD_4:
		if (current_time - patrol_state_timestamp > PATROL_FORWARD_DURATION) {
			patrol_state = PATROL_STATE_TURN_4;
			patrol_state_timestamp = current_time;
			printf("Patrol: Turning 4\r\n");
			smartcar_right();
		}
		break;

	case PATROL_STATE_TURN_4:
		if (current_time - patrol_state_timestamp > PATROL_TURN_DURATION) {
			patrol_state = PATROL_STATE_FORWARD_1; // Cycle complete, restart from the beginning
			patrol_state_timestamp = current_time;
			printf("Patrol: Cycle complete. Restarting patrol.\r\n");
			smartcar_forward(); // Start the next lap
		}
		break;
	}
}

// ---  Servo Control Function ---
void activate_extinguisher_servo(void) {
	BUZZER_Stop();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1200);
	HAL_Delay(300);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2300);
	HAL_Delay(1000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1200);
	HAL_Delay(300);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

// --- Flame Sensor Check and Alert Function (Modified for Patrol Mode) ---
void check_flame_sensor_and_alert(void) {
    static uint8_t flame_state = 0; // 0 = no flame, 1 = flame detected
    uint8_t current_state = (HAL_GPIO_ReadPin(FLAME_SENSOR_GPIO_Port, FLAME_SENSOR_Pin) == GPIO_PIN_SET);

    // Only trigger the main alarm if it's a NEW flame detected DURING patrol.
    if (current_state == 1 && flame_state == 0 && patrol_mode_active) {
        BUZZER_Start();
        HAL_UART_Transmit(&huart3, (uint8_t*)"X\n", 2, 100);
        printf("Flame DETECTED! Alerting PC.\r\n");

        // If flame is detected during patrol, stop the patrol.
        if (patrol_mode_active) {
            patrol_mode_active = 0;
            patrol_state = PATROL_STATE_STOPPED;
            smartcar_stop();
            printf("Patrol stopped due to flame detection.\r\n");
        }
    }
    else if (current_state == 0 && flame_state == 1) {
        BUZZER_Stop();
        printf("Flame cleared.\r\n");
    }
    flame_state = current_state;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	char rx_command = 0;
	uint32_t last_command_time = 0;
	const uint32_t COMMAND_TIMEOUT = 500; // 500ms
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/
	HAL_Init();
	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("RC Car with Flame Sensor Ready!\r\n");
	last_command_time = HAL_GetTick();
	BUZZER_Stop(); // Ensure buzzer is off at start
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// 1. Check for flame sensor activity
		check_flame_sensor_and_alert();

		// 2. Handle patrol mode
		patrol_handler();

		// 3. Handle incoming manual commands from Bluetooth (USART3)
		if (HAL_UART_Receive(&huart3, (uint8_t*) &rx_command, 1, 10)
				== HAL_OK) {
			last_command_time = HAL_GetTick(); // Reset timeout timer

			// Any manual command stops patrol mode
			if (rx_command != 'p' && patrol_mode_active) {
				patrol_mode_active = 0;
				patrol_state = PATROL_STATE_STOPPED;
				printf("Patrol mode deactivated by manual command.\r\n");
			}

			switch (rx_command) {
			case 'w':
				BUZZER_Stop();
				smartcar_forward();
				printf("CMD: Forward\r\n");
				break;
			case 's':
				BUZZER_Stop();
				smartcar_backward();
				printf("CMD: Backward\r\n");
				break;
			case 'a':
				BUZZER_Stop();
				smartcar_left();
				printf("CMD: Left\r\n");
				break;
			case 'd':
				BUZZER_Stop();
				smartcar_right();
				printf("CMD: Right\r\n");
				break;
			case 'j':
				BUZZER_Stop();
				smartcar_stop();
				printf("CMD: Stop\r\n");
				break;
			case 'p': // New command to start patrol
				if (!patrol_mode_active) {
					patrol_mode_active = 1;
					patrol_state = PATROL_STATE_FORWARD_1;
					patrol_state_timestamp = HAL_GetTick();
					printf("Patrol mode activated!\r\n");
					smartcar_forward(); // Start the first move
				}
				break;
			case 'E': // New command for extinguishing
				activate_extinguisher_servo();
				printf("CMD: Extinguish\r\n");
				break;
			default:
				// Other commands (like X from STM32 itself) are ignored here
				break;
			}
		}

		// 4. Timeout safety check for motor (only in manual mode)
		if (!patrol_mode_active
				&& (HAL_GetTick() - last_command_time > COMMAND_TIMEOUT)) {
			if (huart3.RxState == HAL_UART_STATE_READY) {
				smartcar_stop();
				last_command_time = HAL_GetTick();
			}
		}
	}

}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 63;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, RFB_Pin | RBF_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RBB_Pin | LBB_Pin | LBF_Pin | LFB_Pin | LFF_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RFF_Pin */
	GPIO_InitStruct.Pin = RFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RFF_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : FLAME_SENSOR_Pin */
	GPIO_InitStruct.Pin = FLAME_SENSOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(FLAME_SENSOR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RFB_Pin RBF_Pin */
	GPIO_InitStruct.Pin = RFB_Pin | RBF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RBB_Pin LBB_Pin LBF_Pin LFB_Pin
	 LFF_Pin */
	GPIO_InitStruct.Pin = RBB_Pin | LBB_Pin | LBF_Pin | LFB_Pin | LFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
