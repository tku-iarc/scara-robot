/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char test[1] = {0};
uint32_t adc_test = {0, 0};
unsigned char id1_speed[9] = {0xFF,0xFF,0x01,0x05,0x03,0x20,0x64,0x00,0x72};
unsigned char id2_speed[9] = {0xFF,0xFF,0x02,0x05,0x03,0x20,0x64,0x00,0x71};
unsigned char id3_speed[9] = {0xFF,0xFF,0x03,0x05,0x03,0x20,0x0C,0x01,0xC7};
unsigned char id1_move[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0x00,0x00,0x00};//0
unsigned char id2_move[9] = {0xFF,0xFF,0x02,0x05,0x03,0x1E,0x00,0x00,0x00};//0
unsigned char id3_move[9] = {0xFF,0xFF,0x03,0x05,0x03,0x1E,0x00,0x00,0x00};//0
char move_to_0[9] = {0xFF,0xFF,0xFE,0x05,0x03,0x1E,0x00,0x00,0xDB};//0
char move_to_512[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xFF,0x01,0xdb};//512
char move_to_1023[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xFF,0x03,0xd6};//1023
char move_to_2047[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xFF,0x07,0xd5};//2047
char move_to_3041[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xFF,0x0B,0xd1};//3041
char move_to_4095[9] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xFF,0x2F,0xAA};//4095
char move_to_4095_2[9] = {0xFF,0xFF,0x02,0x05,0x03,0x1E,0xFF,0x2F,0xAD};//4095
char set_speed_stop[9] = {0xFF,0xFF,0x01,0x05,0x03,0x20,0x0,0x00,0xd9};//?�度?��0
char set_speed_fast[9] = {0xFF,0xFF,0x01,0x05,0x03,0x20,0xFF,0x03,0xd7};//?�度?��1023
char wheel_mode_speed_fast_forward[9] = {0xFF,0xFF,0xFE,0x05,0x03,0x20,0xFF,0x07,0xd3};//�???????????�???????????
char wheel_mode_speed_fast_reverse[9] = {0xFF,0xFF,0xFE,0x05,0x03,0x20,0xFF,0x03,0xd7};//??��??
char led_on[8] = {0xFF,0xFF,0xFE,0x04,0x03,25,0x01,0xE0};
char led_off[8] = {0xFF,0xFF,0xFE,0x04,0x03,0x19,0x00,0xE1};

char change_to_node_model[11] = {0xFF,0xFF,0xFE,0x07,0x03,0x06,0xFF,0x0F,0xFF,0x0F,0xD5};

char change_to_wheel_model[11] = {0xFF,0xFF,0xFE,0x07,0x03,0x06,0x00,0x00,0x00,0x00,0xF1};

double d_1 = 0.374;
double d_2 = 0;
double d_3 = 0.247;
double l_1 = 0.16458;
double l_2 = 0.085;
double l_3 = 0;
double alpha_1 = 0;
double alpha_2 = 180;
double alpha_3 = 0;
double vaule = 0;
double k1;
double k2;
double A;
double theta_1;
double theta_2;
double theta_3;
double past_theta_1,past_theta_2,past_theta_3;
double clock;
int change_theta_1;
double change_theta_2;
double des_P2_0[4] = {-0.12303658, -0.24303658, 0.05, 1};
int C1;
int C2;
int C3;

void move_to_110xy(double id1_theta ,double id2_theta,int id3_theta)
{
	double id1_fffff;
	int id1_position,id2_position;
	unsigned char id1_low_position,id1_high_position;
	unsigned char id2_low_position,id2_high_position;
	unsigned char id3_low_position,id3_high_position;

	id1_fffff=47;
	id1_position = (id1_theta+id1_fffff)*4096/360*11/2;
	id2_position = id2_theta*4096/360*9/2;

	id1_low_position=id1_position%256;
	id1_high_position=id1_position/256%256;

	id2_low_position=id2_position%256;
	id2_high_position=id2_position/256%256;

	id3_low_position=id3_theta%256;
	id3_high_position=id3_theta/256%256;

	id1_move[6]=id1_low_position;
	id1_move[7]=id1_high_position;
	id1_move[8]=~(id1_move[2]+id1_move[3]+id1_move[4]+id1_move[5]+id1_move[6]+id1_move[7]);

	id2_move[6]=id2_low_position;
	id2_move[7]=id2_high_position;
	id2_move[8]=~(id2_move[2]+id2_move[3]+id2_move[4]+id2_move[5]+id2_move[6]+id2_move[7]);

	id3_move[6]=id3_low_position;
	id3_move[7]=id3_high_position;
	id3_move[8]=~(id3_move[2]+id3_move[3]+id3_move[4]+id3_move[5]+id3_move[6]+id3_move[7]);

	HAL_UART_Transmit_DMA(&huart2, id1_move, 9);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart2, id2_move, 9);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart2, id3_move, 9);
	HAL_Delay(100);
}

void move_to_z(int id3_theta)
{
	unsigned char id3_low_position,id3_high_position;

	id3_low_position=id3_theta%256;
	id3_high_position=id3_theta/256%256;

	id3_move[6]=id3_low_position;
	id3_move[7]=id3_high_position;
	id3_move[8]=~(id3_move[2]+id3_move[3]+id3_move[4]+id3_move[5]+id3_move[6]+id3_move[7]);

	HAL_UART_Transmit_DMA(&huart2, id3_move, 9);
	HAL_Delay(100);
}
void caculate(void)
{
	if(sqrt(pow(des_P2_0[0],2)+pow(des_P2_0[1],2))<=0.24958){
	  if(sqrt(pow(des_P2_0[0],2)+pow(des_P2_0[1],2))>=0.07958){
			A = (pow(des_P2_0[0], 2) + pow(des_P2_0[1], 2) - pow(l_1, 2) - pow(l_2, 2)) / (2 * l_1 * l_2);
		  theta_2 = atan2(sqrt(1 - pow(A, 2)), A);
		  change_theta_2 = theta_2*180/3.14;
		  k1 = l_1 + l_2 * cos(theta_2);
		  k2 = l_2 * sin(theta_2);
		  theta_1 = (atan2(des_P2_0[1] ,des_P2_0[0]) - atan2(k2 ,k1))*180/3.14;

		  theta_3 = des_P2_0[2]*4096/0.0085;

		  theta_1=theta_1;
/*
		  C3 = (theta_3 - past_theta_3)/200;
		  C2 = change_theta_2 - past_theta_2;
		  C1 = theta_1 - past_theta_1;
		  clock=41*(abs(C3)+abs(C2)+abs(C1));

		  past_theta_3=theta_3;
		  past_theta_2=change_theta_2;
		  past_theta_1=theta_1;
		  */
		  move_to_110xy(-theta_1, change_theta_2, theta_3);
		  HAL_Delay(100);
		  //HAL_Delay(clock);
	  }
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      /*初始馬達設定*/
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_UART_Transmit_DMA(&huart2, id1_speed, 9);
	  HAL_Delay(500);
	  HAL_UART_Transmit_DMA(&huart2, id2_speed, 9);
	  HAL_Delay(500);
	  HAL_UART_Transmit_DMA(&huart2, id3_speed, 9);
	  HAL_Delay(500);
	  HAL_UART_Transmit_DMA(&huart2, id1_speed, 9);
	  HAL_Delay(500);
	  HAL_UART_Transmit_DMA(&huart2, change_to_node_model, 11);
	  HAL_Delay(500);
	  move_to_110xy(3,0,0.5);
	  HAL_Delay(2000);
	  HAL_UART_Transmit_DMA(&huart2, led_on, 8);

	  /*LED閃爍3次*/
		  HAL_Delay(1000);
		  HAL_UART_Transmit_DMA(&huart2, led_off, 8);
		  HAL_Delay(1000);
		  HAL_UART_Transmit_DMA(&huart2, led_on, 8);
		  HAL_Delay(1000);
		  HAL_UART_Transmit_DMA(&huart2, led_off, 8);
		  HAL_Delay(1000);
		  HAL_UART_Transmit_DMA(&huart2, led_on, 8);
		  HAL_Delay(1000);
	  while(1){

		/*  move_to_110xy(90,0,0);
		  HAL_Delay(20000);
		  move_to_110xy(-90,0,0);
		  HAL_Delay(10000);
		  move_to_110xy(-90,0,10000);
		  HAL_Delay(10000);
		  move_to_110xy(3,0,10);
		  HAL_Delay(10000);
		  */
	  /*接收資訊*/

	  HAL_UART_Receive(&huart5, test, 1, HAL_MAX_DELAY);
	  HAL_Delay(100);
	  switch(test[0]){
	  case '1':
		  des_P2_0[0] = -0.17647971;
		  des_P2_0[1] = 0.17647971;
		  caculate();
		  caculate();//130.6 10.8
		  //move_to_110xy(-135,0,0.5);

		  break;
	  case '2':
		  des_P2_0[0] = 0;
		  des_P2_0[1] = 0.18647971;
		  caculate();
		  caculate();//54.6 92.5
		  //move_to_110xy(-54.6,92.5,0.5);

		  break;
	  case '3':
		  des_P2_0[0] = 0.17647971;
		  des_P2_0[1] = 0.17647971;
		  caculate();
		  caculate();//40.6 10.8
		  //move_to_110xy(-45,0,0.5);

		  break;
	  case '4':
		  des_P2_0[0] = -0.17647971;
		  des_P2_0[1] = 0;
		  caculate();
		  caculate();//131 88.4
		  //move_to_110xy(-131,88.4,0.5);

		  break;
	  case '5':
		  des_P2_0[0] = 0;
		  des_P2_0[1] = 0;
		  caculate();
		  caculate();//80.9 175.9
		  //move_to_110xy(-90,180,0.5);

		  break;
	  case '6':
		  des_P2_0[0] = 0.17647971;
		  des_P2_0[1] = 0;
		  caculate();
		  caculate();//-19.2 88.4
		  //move_to_110xy(19.2,88.4,0.5);

		  break;
	  case '7':
		  des_P2_0[0] = -0.17647971;
		  des_P2_0[1] = -0.17647971;
		  caculate();
		  caculate();//-139.4 10.8
		  //move_to_110xy(135,0,0.5);

		  break;
	  case '8':
		  des_P2_0[0] = 0;
		  des_P2_0[1] = -0.19647971;
		  caculate();
		  caculate();//-125.4 92.5
		  //move_to_110xy(125.4,92.5,0.5);

		  break;
	  case '9':
		  des_P2_0[0] = 0.17647971;
		  des_P2_0[1] = -0.17647971;
		  caculate();
		  caculate();//-49.4 10.8
		  //move_to_110xy(45,0,0.5);

		  break;

	  case 'u':

		  des_P2_0[2] =des_P2_0[2]+0.01 ;
		  caculate();
		  caculate();

		  break;
	  case 'd':
		  des_P2_0[2] =des_P2_0[2]-0.01;
		  caculate();
		  caculate();

		  break;
	  case 'f':
		  des_P2_0[1] =des_P2_0[1]+0.01;
		  caculate();
		  caculate();
		  break;
	  case 'b':
		  des_P2_0[1] = des_P2_0[1]-0.01;
		  caculate();
		  caculate();
		  break;
	  case 'l':
		  des_P2_0[0] = des_P2_0[0]-0.01;
		  caculate();
		  caculate();

		  break;
	  case 'r':
		  des_P2_0[0] = des_P2_0[0]+0.01;
		  caculate();
		  caculate();

		  break;
	  case 'z':
		  des_P2_0[0] = 0.24958;
		  des_P2_0[1] = 0;
		  des_P2_0[2] = 0.5;
		  caculate();
		  caculate();

		  break;
	  default:
		  des_P2_0[0] = des_P2_0[0];
		  des_P2_0[1] = des_P2_0[1];
		  des_P2_0[2] = des_P2_0[2];
		  break;

	  }
	  test[0]='0';

	  /*3點測試*/

/*


//	  theta_3=10;
//	  change_theta_2=0;
//	  theta_1=3;
//
//	  C3 = (theta_3 - past_theta_3)/200;
//	  C2 = change_theta_2 - past_theta_2;
//	  C1 = theta_1 - past_theta_1;
//	  clock=40*(abs(C3)+abs(C2)+abs(C1));
//	  HAL_Delay(clock);
//	  past_theta_3=10;
//	  past_theta_2=0;
//	  past_theta_1=3;

	   des_P2_0[0] = 0.12303658;
	   des_P2_0[1] = -0.24303658;
	   des_P2_0[2] =0.05;
	   caculate();
	   HAL_Delay(10000);

	   HAL_UART_Transmit_DMA(&huart2, led_off, 8);
	   HAL_Delay(1000);
	   HAL_UART_Transmit_DMA(&huart2, led_on, 8);
	   HAL_Delay(1000);

	   des_P2_0[0] = 0.064;
	   des_P2_0[1] = 0;
	   des_P2_0[2] =0.01;
	   caculate();
	   HAL_Delay(10000);

	   move_to_110xy(3,0,0);
	   HAL_Delay(10000);
*/
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 1000000;
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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(data_control_GPIO_Port, data_control_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : data_control_Pin */
  GPIO_InitStruct.Pin = data_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(data_control_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
