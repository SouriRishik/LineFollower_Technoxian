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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#define address 1
#define kp 8.8
#define kd 0.5
#define ki 0.000004
#define MAX_LOOP_COUNT 4

double error, P, I, D;
int setpoint = 35;
double correction = 0, lastInput = 0;
uint32_t lastTime = 0;
double integralMin = -25.0, integralMax = 25.0;
//double base_speed_right = 150;
//double base_speed_left = 150;
double base_speed = 200;
int turn;
int junction;
int junction_flag = 0;
int loop_counter = 0;
int last_turn = 0;
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[1];
char rxdata[20];
int flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void send_to_lsa(uint8_t command, uint8_t data) {
	uint8_t packet[4];
	packet[0] = address;
	packet[1] = command;
	packet[2] = data;
	packet[3] = address + command + data;
	HAL_UART_Transmit(&huart1, packet, 4, 10);
	HAL_Delay(10);

}

int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

const int weights[8] = {0, 10, 20, 30, 40, 50, 60, 70};

uint8_t line_data() {
    uint8_t rxByte = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
    HAL_Delay(1);
    HAL_UART_Receive(&huart1, &rxByte, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
    return rxByte;
}

int process_byte(uint8_t byte) {
    int weighted_sum = 0;
    int count_active_sensors = 0;

    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            weighted_sum += weights[i];
            count_active_sensors++;
        }
    }

    if (count_active_sensors == 0) {
        return 255;
    }

    return weighted_sum / count_active_sensors;
}


int junction_data() {
	int j_data = HAL_GPIO_ReadPin(JPULSE_GPIO_Port, JPULSE_Pin);
	return j_data;
}

void setMotorSpeed(uint8_t motor, int32_t speed) {
	uint16_t pwm = abs(speed);
//	printf("in motor speed");
	if (pwm > 200)
		pwm = 200;

	if (motor == 0) { // Motor 1
		if (speed > 0) {
			TIM2->CCR1 = pwm;  // Set PWM duty cycle for channel 1
			TIM3->CCR1 = 0;          // Set PWM duty cycle for channel 2
		} else {
			TIM2->CCR1 = 0;          // Set PWM duty cycle for channel 1
			TIM3->CCR1 = pwm;  // Set PWM duty cycle for channel 2
		}
	} else if (motor == 1) { // Motor 2
		if (speed > 0) {
			TIM3->CCR3 = pwm;  // Set PWM duty cycle for channel 3
			TIM3->CCR4 = 0;          // Set PWM duty cycle for channel 4
		} else {
			TIM3->CCR3 = 0;          // Set PWM duty cycle for channel 3
			TIM3->CCR4 = pwm;  // Set PWM duty cycle for channel 4
		}
	}

}

void computePID(double error, int32_t input) {

	double timeChange = (double) (HAL_GetTick() - lastTime);
//	printf("Inside compute pid error = %f\n", error);
	P = kp * error;
	I += ki * error * timeChange;
	if (I > integralMax) I = integralMax;
	if (I < integralMin) I = integralMin;
	D = kd * (input - lastInput) / timeChange;

	correction = P + I + D;
	lastInput = input;
	lastTime = HAL_GetTick();
	correction = floor(correction);
//	base_speed_right = floor(base_speed_right);
//	base_speed_left = floor(base_speed_left);

//	printf("input = %d\n", input);
//	printf("lastinput = %lf\n", lastInput);
//	printf("P = %f\n", P);
//	printf("I = %f\n", I);
//	printf("D = %f\n", D);
////	base_speed_right -= correction;
////	base_speed_left += correction;
//	printf("correction = %f\n", correction);
////	printf("motor0: %f, motor1:%f\n" ,base_speed_left,base_speed_right);
////	setMotorSpeed(0, base_speed_left);
////	setMotorSpeed(1, base_speed_right);
//	printf("motor0: %f, motor1:%f\n", base_speed + correction,
//			base_speed - correction);
	setMotorSpeed(0, base_speed + correction);
	setMotorSpeed(1, base_speed - correction);

}
uint8_t rxData;

//int isInLoop(int current_turn) {
//    if (current_turn == last_turn) {
//        loop_counter++;
//        if (loop_counter >= MAX_LOOP_COUNT) {
//            return 1;
//        }
//    } else {
//        loop_counter = 0;
//    }
//    last_turn = current_turn;
//    return 0;
//}

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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
//    send_to_lsa('C', 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_Delay(6000);
	HAL_Delay(2000);
//  setMotorSpeed(0, base_speed);
//  setMotorSpeed(1, base_speed);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		int input_byte = line_data();
		int input = process_byte(input_byte);
//		printf("Input: %d\n", input);
//		printf("input byte: %d\n", input_byte);
//		printf("Turn: %d\n", turn);
//		printf("junction: %d\n", junction);
		turn = input > 50 && input <= 70 ? 1 : input < 20 ? -1 : turn;
//		if (input > 52 && input <= 70) {
//		        turn = 1;
//		} else if (input < 20) {
//		        turn = -1;
//		}
//		if (isInLoop(turn)) {
//		    if (turn == 1) {
//		        turn = -1;
//		    } else if (turn == -1) {
//		        turn = 1;
//		    }
//		}

		if (turn) {
			while (input == 255) {
//				printf("here");
				if (turn == 1) {
					setMotorSpeed(0, 150);
					setMotorSpeed(1, -150);
				} else if(turn == -1) {
					setMotorSpeed(0, -150);
					setMotorSpeed(1, 150);
				}
				input_byte = line_data();
				input = process_byte(input_byte);
			}
//			turn = 0;
//			continue;
//			input = line_data();
			input_byte = line_data();
			input = process_byte(input_byte);
		}
//		turn = 0;
		turn = input > 52 && input <= 70 ? 1 : input < 20 ? -1 : turn;
		if (input == 255) {
//			setMotorSpeed(0, 0);
//			setMotorSpeed(1, 0);
			continue;
		}
//		junction = junction_data();
//		if (junction == 1) {
//			turn = input > 45 ? 1 : input < 25 ? -1 : 0;
//			continue;
//		}`

//
//		if (junction) {
//			if (junction_flag == 0) {
//				if (input > 45) {
//					turn = 1;
//				} else if (input < 25) {
//					turn = -1;
//				} else {
//					turn = 0;
//				}
//
//				junction_flag = 1;
//			} else {
//				turn = 0;
//			}
//		} else {
//			junction_flag = 0;
//		}
//		if (input == 255) {
//			HAL_Delay(25);
//			setMotorSpeed(0, 0);
//			setMotorSpeed(1, 0);
//			while (line_data() == 255) {
//				if (turn == 1) {
//					setMotorSpeed(0, 50);
//					setMotorSpeed(1, -50);
//					printf("turn=1");
//				} else if (turn == -1) {
//					setMotorSpeed(0, -50);
//					setMotorSpeed(1, 50);
//					printf("turn=-1");
//				} else if (!turn) {
//					setMotorSpeed(0, 50);
//					setMotorSpeed(1, 50);
//					printf("turn=0");
//				}
//				HAL_Delay(200);
//			}
//			input = line_data();
//		}
//	else{
		double error = input - setpoint;
//		if(turn){
//			error = error * 1.5;
//		}
		computePID(error, input);
//		}
//	}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//	if(flag == 1){
//
//		HAL_UART_Receive(&huart2, (uint8_t *)rxdata, sizeof(rxdata), 1);
//		for (int i = 0; i < strlen(rxdata); i++) {
//			printf("%c,", rxdata[i]); //	function to convert string to float to be called here
//		}
//		printf("\n");
//		flag = 0;
//		__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
//
//	}
//		printf("line: %d\n", line_data());

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	htim2.Init.Prescaler = 21 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 200;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 21 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 200;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GND_Pin | UEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GND1_GPIO_Port, GND1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : GND_Pin UEN_Pin */
	GPIO_InitStruct.Pin = GND_Pin | UEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : GND1_Pin */
	GPIO_InitStruct.Pin = GND1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GND1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : JPULSE_Pin */
	GPIO_InitStruct.Pin = JPULSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(JPULSE_GPIO_Port, &GPIO_InitStruct);

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
