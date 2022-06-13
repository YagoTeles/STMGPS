/* USER CODE BEGIN Header */
/**
  ****
  * @file           : main.c
  * @brief          : Main program body
  ****
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "RingBuffer.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  BIT0   (1 << 0)
#define  BIT1   (1 << 1)
#define  BIT2   (1 << 2)
#define  BIT3   (1 << 3)
#define  BIT4   (1 << 4)
#define  BIT5   (1 << 5)
#define  BIT6   (1 << 6)
#define  BIT7   (1 << 7)
#define  BIT8   (1 << 8)
#define  BIT9   (1 << 9)
#define  BIT10  (1 << 10)
#define  BIT11  (1 << 11)
#define  BIT12  (1 << 12)
#define  BIT13  (1 << 13)
#define  BIT14  (1 << 14)
#define  BIT15  (1 << 15)

#define UART_RX_BUFFER_SIZE 100

#define  EVENTO_BOTAO   BIT0
#define  EVENTO_LDR     BIT1
#define  EVENTO_DS18B20 BIT2



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId TarefaBotaoHandle;
osThreadId TarefaPcHandle;
osThreadId TarefaEspHandle;
osThreadId TarefaLdrHandle;
osThreadId TarefaGpsHandle;
osThreadId TarefaDs18b20Handle;
osThreadId TarefaServidorHandle;
/* USER CODE BEGIN PV */
static uint8_t         rxBufferEsp[UART_RX_BUFFER_SIZE];  // Vetor usado para o Buffer circular do ESP8266  
static cbuf_handle_t   ringBufferEsp;         // Estrutura do buffer circular do ESP8266
static uint8_t         byteReceivedEsp;       // Vari?vel que recebe byte por interrup??o do ESP8266
                                              // Retira do buffer para byteFromBufferEsp
static uint8_t         rxBufferPc[UART_RX_BUFFER_SIZE]; // Vetor usado para o Buffer circular do PC
static cbuf_handle_t   ringBufferPc;          // Estrutura do buffer circular do PC
static uint8_t         byteReceivedPc;        // Vari?vel que recebe byte por interrup??o do PC

static uint8_t         rxBufferGps[UART_RX_BUFFER_SIZE]; // Vetor usado para o Buffer circular do GPS
static cbuf_handle_t   ringBufferGps;          // Estrutura do buffer circular do GPS
static uint8_t         byteReceivedGps;        // Vari?vel que recebe byte por interrup??o do GPS

EventGroupHandle_t grupoDeEventos;

static char bufferGps[80] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void TrataBotao(void const * argument);
void TrataPc(void const * argument);
void TrataEsp(void const * argument);
void TrataLdr(void const * argument);
void TrataGps(void const * argument);
void TrataDs18b20(void const * argument);
void TrataServidor(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  grupoDeEventos = xEventGroupCreate();
	
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TarefaBotao */
  osThreadDef(TarefaBotao, TrataBotao, osPriorityNormal, 0, 128);
  TarefaBotaoHandle = osThreadCreate(osThread(TarefaBotao), NULL);

  /* definition and creation of TarefaPc */
  osThreadDef(TarefaPc, TrataPc, osPriorityNormal, 0, 128);
  TarefaPcHandle = osThreadCreate(osThread(TarefaPc), NULL);

  /* definition and creation of TarefaEsp */
  osThreadDef(TarefaEsp, TrataEsp, osPriorityNormal, 0, 128);
  TarefaEspHandle = osThreadCreate(osThread(TarefaEsp), NULL);

  /* definition and creation of TarefaLdr */
  osThreadDef(TarefaLdr, TrataLdr, osPriorityNormal, 0, 128);
  TarefaLdrHandle = osThreadCreate(osThread(TarefaLdr), NULL);

  /* definition and creation of TarefaGps */
  osThreadDef(TarefaGps, TrataGps, osPriorityNormal, 0, 128);
  TarefaGpsHandle = osThreadCreate(osThread(TarefaGps), NULL);

  /* definition and creation of TarefaDs18b20 */
  osThreadDef(TarefaDs18b20, TrataDs18b20, osPriorityNormal, 0, 128);
  TarefaDs18b20Handle = osThreadCreate(osThread(TarefaDs18b20), NULL);

  /* definition and creation of TarefaServidor */
  osThreadDef(TarefaServidor, TrataServidor, osPriorityNormal, 0, 128);
  TarefaServidorHandle = osThreadCreate(osThread(TarefaServidor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedVerde_GPIO_Port, LedVerde_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ds18b20_GPIO_Port, Ds18b20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Botao3_Pin Botao1_Pin Botao2_Pin */
  GPIO_InitStruct.Pin = Botao3_Pin|Botao1_Pin|Botao2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LedVerde_Pin */
  GPIO_InitStruct.Pin = LedVerde_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedVerde_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ds18b20_Pin */
  GPIO_InitStruct.Pin = Ds18b20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Ds18b20_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t acordouTarefa = pdFALSE;
    
    if (GPIO_Pin == BIT13)
    {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        vTaskNotifyGiveFromISR(TarefaBotaoHandle, &acordouTarefa);
        portYIELD_FROM_ISR(acordouTarefa);
    }
}


/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
int cont = 0;
int hr=0,min=0,dia=0,mes=0,ano=0;
void ProcessRMC()
	{
		char time[80];
		char pos[1];
		char lat[20], lon[20];
		char lonDir[80], latDir[80];
		char speed[80];
		char track[80];
		char date[80];
		char magVar[80];
		char varDir[80];
		char mode[1];
		char checkSum[3];
		
		//double latTemp, latGrau, latMinutos, latSegundos, lonTemp, lonGrau, lonSegundos;
		char latitude[40], latGrau[20], latMinutos[20], latSegundos[20], longitude[40],lonMinutos[20], lonGrau[20], lonSegundos[20], timeTotal[40], timeHora[20],timeMin[20], timeSeg[20], dia[20], mes[20], ano[20], dataDeEnvio[40];
		
		//double latEnviar;
		
		uint8_t result = 0;
		result = sscanf(bufferGps, "$GPRMC,%[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^','], %[^',']", time, pos, lat, latDir, lon, lonDir, speed, track, date, magVar, varDir, mode, checkSum);
		

		HAL_UART_Transmit(&huart2, (uint8_t *)date, 20, 10);
		
		char string1[5] = "\n";
		if(result)
		{
			//Latitude_______________________________________________________________________________________________________________________________________________________________________________________________________
			strcat(latitude,"Latitude: ");
            
      strcat(latGrau,&lat[0]); 
      strcat(latGrau,&lat[1]);
      latGrau[2] = '\0';
      strcat(latitude,latGrau);
      strcat(latitude,"°");

      strcat(latMinutos,&lat[2]); 
      strcat(latMinutos,&lat[3]); 
      latMinutos[2] = '\0';
			strcat(latitude,latMinutos);
      strcat(latitude,"'");

      strcat(latSegundos,&lat[5]); 
      strcat(latSegundos,&lat[6]); 
      latSegundos[2] = '\0';
			strcat(latitude,latSegundos);
      strcat(latitude,"''");
			strcat(latitude,latDir);
			strcat(latitude,"\n");
			
			HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)latitude, strlen(latitude), 10);
			
			strcpy(latGrau, "\0");
			strcpy(latMinutos, "\0");
			strcpy(latSegundos, "\0");
			strcpy(latitude, "\0");
			
			//Longitude_______________________________________________________________________________________________________________________________________________________________________________________________________
			strcat(longitude,"Longitude: ");
            
      strcat(lonGrau,&lon[0]); 
      strcat(lonGrau,&lon[1]);
      lonGrau[2] = '\0';
      strcat(longitude,lonGrau);
      strcat(longitude,"°");

      strcat(lonMinutos,&lon[2]); 
      strcat(lonMinutos,&lon[3]); 
      lonMinutos[2] = '\0';
			strcat(longitude,lonMinutos);
      strcat(longitude,"'");

      strcat(lonSegundos,&lon[4]); 
      strcat(lonSegundos,&lon[6]); 
      lonSegundos[2] = '\0';
			strcat(longitude,lonSegundos);
      strcat(longitude,"''");
			strcat(longitude,lonDir);
			strcat(longitude,"\n");
			
			HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)longitude, strlen(longitude), 10);
			
			strcpy(lonGrau, "\0");
			strcpy(lonMinutos, "\0");
			strcpy(lonSegundos, "\0");
			strcpy(longitude, "\0");
			
			//Tempo___________________________________________________________________________________________________________________________________________________________________________________________________________
			strcat(timeTotal,"Hora: ");
            
      strcat(timeHora,&time[0]); 
      strcat(timeHora,&time[1]);
      timeHora[2] = '\0';
      strcat(timeTotal,timeHora);
      strcat(timeTotal,"h");

      strcat(timeMin,&time[2]); 
      strcat(timeMin,&time[3]); 
      timeMin[2] = '\0';
			strcat(timeTotal,timeMin);
      strcat(timeTotal,"m");
			
      strcat(timeSeg,&time[4]); 
      strcat(timeSeg,&time[5]); 
      timeSeg[2] = '\0';
			strcat(timeTotal,timeSeg);
      strcat(timeTotal,"s");
			strcat(timeTotal,"\n");
			
			HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)timeTotal, 20, 10);
			
			strcpy(timeHora, "\0");
			strcpy(timeMin, "\0");
			strcpy(timeSeg, "\0");
			strcpy(timeTotal, "\0");
			
			//Data____________________________________________________________________________________________________________________________________________________________________________________________________________
			
			strcat(dataDeEnvio,"Data: ");
            
      strcat(dia,&date[0]); 
      strcat(dia,&date[1]);
      dia[2] = '\0';
      strcat(dataDeEnvio,dia);
      strcat(dataDeEnvio,"/");

      strcat(mes,&date[2]); 
      strcat(mes,&date[3]); 
      mes[2] = '\0';
			strcat(dataDeEnvio,mes);
      strcat(dataDeEnvio,"/");
			
      strcat(ano,&date[4]); 
      strcat(ano,&date[5]); 
      ano[2] = '\0';
			strcat(dataDeEnvio,ano);
			strcat(dataDeEnvio,"\n");
			
			HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)dataDeEnvio, 20, 10);
			
			strcpy(dia, "\0");
			strcpy(mes, "\0");
			strcpy(ano, "\0");
			strcpy(dataDeEnvio, "\0");
	
			
		}

	}
uint8_t GPS_recebeByte(uint8_t byteReceived){
		//circular_buf_put(ringBufferGps, byteReceivedGps);              // put every received byte into ring buffer
		//HAL_UART_Receive_IT(&huart1, &byteReceivedGps, 1);
		static uint8_t indice = 0;
		static uint8_t estado = 0;	
		uint8_t result = 0;
	
		bufferGps[indice++] = byteReceived;
		
		switch(estado)
		{
			case 0:
				if(byteReceived == '$')
				{
					estado++;
					indice = 1;
				}
				else 
				{
					indice = 0;
				}
				break;
			
			case 1:
				if(byteReceived == '*')
				{
					estado++;
				}
								
				break;
			
			case 2:
				estado++;
				break;
			
			case 3:
				estado++;
				break;
			
			case 4:
				if(byteReceived == '\n')
				{
					if (1)
						//(0 == strncmp(bufferGps, "$GPRMC", 6))
					{
						ProcessRMC();
						estado = 0; //deletar
						indice = 0;
						result = 1;
					}
				}
				break;
			default:
				indice = 0;
				estado = 0;
				//strcpy(bufferGps, "");
		}
		//strcpy(bufferGps, "");
		return result;
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		char tempChar;
    BaseType_t acordouTarefa = pdFALSE;

    if (huart == &huart1)//mudar pra 2 dnv!!!!!!!!!!!!!!!!!!!
    {
        circular_buf_put(ringBufferPc, byteReceivedPc);              // put every received byte into ring buffer
        HAL_UART_Receive_IT(&huart2, &byteReceivedPc, 1);            // Set up next byte interrupt
        vTaskNotifyGiveFromISR(TarefaPcHandle, &acordouTarefa);
        portYIELD_FROM_ISR(acordouTarefa);
    }
    else if (huart == &huart6)
    {
        circular_buf_put(ringBufferEsp, byteReceivedEsp);              // put every received byte into ring buffer
        HAL_UART_Receive_IT(&huart6, &byteReceivedEsp, 1);
        vTaskNotifyGiveFromISR(TarefaEspHandle, &acordouTarefa);
        portYIELD_FROM_ISR(acordouTarefa);
    }
    else if (huart == &huart2)//mudar pra 1 dnv!!!!!!!!!!!!!!!!!!!!!!!!!!
    {
			tempChar = byteReceivedGps;
			HAL_UART_Receive_IT(&huart2, &byteReceivedGps, 1);
      if (GPS_recebeByte(tempChar))
				{
					vTaskNotifyGiveFromISR(TarefaGpsHandle, &acordouTarefa);
					portYIELD_FROM_ISR(acordouTarefa);
				}
    }
 }



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LedVerde_GPIO_Port, LedVerde_Pin);
      osDelay(250);  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TrataBotao */
/**
* @brief Function implementing the TarefaBotao thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataBotao */
void TrataBotao(void const * argument)
{
  /* USER CODE BEGIN TrataBotao */

  /* Infinite loop */
  for(;;)
  {
  		  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xEventGroupSetBits(grupoDeEventos, EVENTO_BOTAO);

//   Filtro de debounce
        osDelay(100);
        EXTI->PR = BIT13;
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
  /* USER CODE END TrataBotao */
}

/* USER CODE BEGIN Header_TrataPc */
/**
* @brief Function implementing the TarefaPc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataPc */
void TrataPc(void const * argument)
{
  /* USER CODE BEGIN TrataPc */
     static uint8_t byteFromBufferPc;

    ringBufferPc = circular_buf_init(rxBufferPc, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart2, &byteReceivedPc, 1);

    /* Infinite loop */
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (circular_buf_get(ringBufferPc, &byteFromBufferPc) == SUCCESS) // While there is byte to be received,
        {
            HAL_UART_Transmit(&huart6, &byteFromBufferPc, 1, 10);
        }
    }

  /* USER CODE END TrataPc */
}

/* USER CODE BEGIN Header_TrataEsp */
/**
* @brief Function implementing the TarefaEsp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataEsp */
void TrataEsp(void const * argument)
{
  /* USER CODE BEGIN TrataEsp */
    static uint8_t byteFromBufferEsp;
    
    ringBufferEsp = circular_buf_init(rxBufferEsp, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart6, &byteReceivedEsp, 1);

    /* Infinite loop */
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
        while (circular_buf_get(ringBufferEsp, &byteFromBufferEsp) == SUCCESS) // While there is byte to be received,
        {
            HAL_UART_Transmit(&huart2, &byteFromBufferEsp, 1, 10);
					
					
        }
    }
  /* USER CODE END TrataEsp */
}

/* USER CODE BEGIN Header_TrataLdr */
/**
* @brief Function implementing the TarefaLdr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataLdr */
void TrataLdr(void const * argument)
{
  /* USER CODE BEGIN TrataLdr */
    char string2[30];
    uint32_t valorAdc;
    
  /* Infinite loop */
  for(;;)
  {
      HAL_ADC_Start(&hadc1);
      if ( HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
      {
          valorAdc = HAL_ADC_GetValue(&hadc1);
          sprintf(string2, "Adc = %0X\n", valorAdc);
//          HAL_UART_Transmit(&huart2, (uint8_t *)string2, strlen(string2), 10);
   	      xEventGroupSetBits(grupoDeEventos, EVENTO_LDR); 
      }
      osDelay(2000);
		}
  /* USER CODE END TrataLdr */
}

/* USER CODE BEGIN Header_TrataGps */
/**
* @brief Function implementing the TarefaGps thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataGps */
void TrataGps(void const * argument)
{
  /* USER CODE BEGIN TrataGps */
    static uint8_t byteFromBufferGps;

   // ringBufferGps = circular_buf_init(rxBufferGps, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart2, &byteReceivedGps, 1);//mudar pra uart 1
	
		char string1[] = "\nRecebeu msg\n";

    /* Infinite loop */
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			
				HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
				HAL_UART_Transmit(&huart2, (uint8_t *)bufferGps, strlen(bufferGps), 10);
        /*while (circular_buf_get(ringBufferGps, &byteFromBufferGps) == SUCCESS) // While there is byte to be received,
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
						HAL_UART_Transmit(&huart2, (uint8_t *)bufferGps, strlen(bufferGps), 10);
        }*/
    }
  /* USER CODE END TrataGps */
}

/* USER CODE BEGIN Header_TrataDs18b20 */
/**
* @brief Function implementing the TarefaDs18b20 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataDs18b20 */
void TrataDs18b20(void const * argument)
{
  /* USER CODE BEGIN TrataDs18b20 */
  /* Infinite loop */
  for(;;)
  {
      xEventGroupSetBits(grupoDeEventos, EVENTO_DS18B20);
      osDelay(1000);
  }
  /* USER CODE END TrataDs18b20 */
}

/* USER CODE BEGIN Header_TrataServidor */
/**
* @brief Function implementing the TarefaServidor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrataServidor */
void TrataServidor(void const * argument)
{
  /* USER CODE BEGIN TrataServidor */
	  const EventBits_t eventosAguardados = EVENTO_BOTAO | 
			                                    EVENTO_DS18B20 | 
		                                      EVENTO_LDR;
	  EventBits_t eventos;
    char string1[] = "Apertou botao!\n";
    char string2[] = "Sensor de temperatura!\n";
    char string3[] = "Transdutor LDR!\n";
  /* Infinite loop */
  for(;;)
  {
      eventos = xEventGroupWaitBits(grupoDeEventos,
																		eventosAguardados,
																		pdTRUE,
																		pdTRUE,
																		portMAX_DELAY);
		    if ( (eventos & EVENTO_BOTAO) != 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t *)string1, strlen(string1), 10);
				}
				if ( (eventos & EVENTO_DS18B20) != 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t *)string2, strlen(string2), 10);
				}
				if ( (eventos & EVENTO_LDR) != 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t *)string3, strlen(string3), 10);
				}
  }
  /* USER CODE END TrataServidor */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
