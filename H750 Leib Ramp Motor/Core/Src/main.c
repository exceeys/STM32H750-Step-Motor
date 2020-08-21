/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "quadspi.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "StepperMotor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE				20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char* HelloMessage = "STM32H750 Leib Ramp V0.1\n";

float RxNumber = 0;
char Str[30] = "Pos :   ";
uint8_t UART_RxBuffer[10];
uint8_t UART_RxCmd[UART_RX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SendOK(void)
{
	CDC_Transmit_FS((uint8_t*)("OK\n"), 3);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)("OK\n"), 3);
}

// 현재 위치를 통신포트로 전송한다.
void DisplayPosition(void)
{
	//sprintf(Str, "Pos : %9.2f\n", GetPosition());
	sprintf(Str, "Pos : %.3f\n", GetPosition());

	CDC_Transmit_FS((uint8_t *)Str, strlen(Str));
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)Str, strlen(Str));
}

void DisplayInfo(void)
{
	char InfoStr[100];

	sprintf(InfoStr, "Acc : %ld\nSpd : %.3f\nPos : %.3f\n", GetAcceleration(), GetSpeed(), GetPosition());
	CDC_Transmit_FS((uint8_t *)InfoStr, strlen(InfoStr));
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)InfoStr, strlen(InfoStr));
}

// 인터럽트 우선순위를 모터 타이머 보다 낮게 설정한다.
// 안그러면 모터가 틱틱거리기도 한다.
void HAL_SYSTICK_Callback(void)
{
	static int cnt = 0;

	cnt++;
	if(cnt > 500)
	{
		cnt = 0;

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		if(GetState() == STATE_BUSY)
		{
			DisplayPosition();
		}
	}
}

// usbd_cdc_if.c에서 데이터가 수신되면 불려지게 함
// 새로운 명령어(기능)을 추가하려면 여기에서 명령어를 추가하여 인식하게 한다.
void ExecuteCommand(uint8_t* Buf, uint32_t len)
{
	int Sign = 1, Point = 0, i;
	uint8_t cmd = 0;

	for(i = 0; i < len; i++ )
	{
    if((Buf[i] >= 'a') && (Buf[i] <= 'z')) Buf[i] = Buf[i] - 'a' + 'A';		// 대문자로 바꿈

		// 명령어 종류를 구분
		switch(Buf[i])
		{
			case 'A':	// 가속도 설정
			case 'S':	// 속도 설정
			case 'L':	// 현위치 재설정
			case 'G':	// 현위치 표시
			case 'D':	// 상대 위치 이동
			case 'P':	// 절대 위치 이동
			case 'E':	// 즉시 정지
			case 'I':	// 현재 설정 상태 표시
      cmd = Buf[i];
				break;

			default:
				continue;
		}
		if(cmd != 0) break;
	}
	if(i == len)	// 스트링안에 인식가능 명령어가 없슴.
	{
		CDC_Transmit_FS((uint8_t*)("NG\n"), 3);
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)("NG\n"), 3);

		return;
	}

	// 전달된 값을 계산(문자열을 숫자로 바꿈)
	float Pow = 0.1f;
	RxNumber = 0.0f;
	for(; i < len; i++)
	{
		if(Buf[i] == '-') Sign = -1;
		if(Buf[i] == '.') Point = 1;
		if(Buf[i] < '0' || Buf[i] > '9') continue;

		if(Point)
		{
			//RxNumber += (Buf[i] - '0') * powf(0.1, Point);	// powf() 가 큰 함수라서 대체
			//Point++;
			RxNumber += (Buf[i] - '0') * Pow;
			Pow *= 0.1f;
		}
		else
		{
			RxNumber *= 10.0f;
			RxNumber += Buf[i] - '0';
		}
	}
	RxNumber *= Sign;

	// 명령어 실행
	switch(cmd)
	{
		case 'A':			// 가속도 설정
			SetAcceleration(RxNumber);
			SendOK();		// 즉시 처리 명령어 이므로 바로 응답함
			break;

		case 'S':			// 속도 설정
			SetSpeed(RxNumber);
      SendOK();
			break;

		case 'L':			// 현위치 재설정
			SetPosition(RxNumber);
      SendOK();
			break;

		case 'G':			// 현위치 표시
			DisplayPosition();
			break;

		case 'D':			// 상대 위치 이동
			MoveDistance(RxNumber);
			break;

		case 'P':			// 절대 위치 이동
			MovePosition(RxNumber);
			break;

		case 'E':			// 즉시 정지
			StopEMS();
      SendOK();
			break;

		case 'I':			// 현재 설정 상태 표시
			DisplayInfo();
			break;
	}
}

// 동작이 끝나면 불려지는 콜백 함수
void MoveEndCallback(void)
{
	sprintf(Str, "Pos : %.3f\nOK\n", GetPosition());
	CDC_Transmit_FS((uint8_t *)Str, strlen(Str));
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)Str, strlen(Str));
}

// USART에서 명령이 오면 실행한다.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static int pos = 0;

  UART_RxCmd[pos] = UART_RxBuffer[0];
  if(pos < UART_RX_BUFFER_SIZE)
  {
  	pos++;
  }

  if(UART_RxBuffer[0] == '\n')
  {
  	ExecuteCommand(UART_RxCmd, pos);
  	pos = 0;
  }

  HAL_UART_Receive_IT(&huart1, UART_RxBuffer, 1);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  InitStepperMotor(25, 100);
  EnableMotor();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)HelloMessage, strlen(HelloMessage));
  HAL_UART_Receive_IT(&huart1, UART_RxBuffer, 1);   // USART1으로 1바이트씩 받자

  // USART으로 부터 명령이 들어오므로 RX 인터럽트 서비스 루틴인 HAL_UART_RxCpltCallback() 함수에서
  // 모든 것이 시작된다.  그러므로 main()의 무한루프에서는 할 것이 없다.
  // 또는 USB CDC로 사용할 경우 usbd_cdc_if.c 에 있는 CDC_Receive_FS(...) 함수에서 하나의 명령 문장이
  // 들어오면 ExecuteCommand()로 보내 처리하게 했다.

  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 30;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 10;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */

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
