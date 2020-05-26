/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#define PULSE_PER_CYCLE 600
#define WIDTH_OF_STEP		(PULSE_PER_CYCLE/6)
#define TIM_PWM 	htim1

#define SW1_A8_PWM_MODE_CH1				MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF8_Msk, 0x00000002 << GPIO_CRH_CNF8_Pos)
#define SW2_A9_PWM_MODE_CH2				MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF9_Msk, 0x00000002 << GPIO_CRH_CNF9_Pos)
#define SW3_A10_PWM_MODE_CH3			MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF10_Msk,0x00000002 << GPIO_CRH_CNF10_Pos)

#define SW4_A7_PWM_MODE_CH1N			MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_Msk, 0x00000002 << GPIO_CRL_CNF7_Pos)
#define SW5_B0_PWM_MODE_CH2N			MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0_Msk, 0x00000002 << GPIO_CRL_CNF0_Pos)
#define SW6_B1_PWM_MODE_CH3N			MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF1_Msk, 0x00000002 << GPIO_CRL_CNF1_Pos)

#define SW1_A8_GPIO_MODE					MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF8_Msk, 0)
#define SW2_A9_GPIO_MODE					MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF9_Msk, 0)
#define SW3_A10_GPIO_MODE					MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF10_Msk,0)

#define SW4_A7_GPIO_MODE					MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_Msk, 0)
#define SW5_B0_GPIO_MODE					MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0_Msk, 0)
#define SW6_B1_GPIO_MODE					MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF1_Msk, 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int16_t i=0,
				j=4*WIDTH_OF_STEP,
				k=2*WIDTH_OF_STEP;
				/*
int16_t VREF[PULSE_PER_CYCLE] = {
	225,	447,	662,	867,	1058,	1232,	1386,	1519,	1628,	1711,	1768,	1796,
	1796,	1768,	1711,	1628,	1519,	1386,	1232,	1058,	867,	662,	447,	225,
	-226,	-448,	-663,	-868,	-1059,	-1233,	-1387,	-1520,	-1629,	-1712,	-1768,	-1796,
	-1796,	-1768,	-1712,	-1629,	-1520,	-1387,	-1233,	-1059,	-868,	-663,	-448,	-226 };*/
int temp =0;
				
int16_t VREF[PULSE_PER_CYCLE] = {0,  19,  38,  57,  75,  94,  113,  132,  151,  169,  188,  207,  226,  244,  263,  282,  300,
  319,  337,  356,  374,  393,  411,  429,  448,  466,  484,  502,  520,  538,  556,  574,  592,  610,  627, 
	645,  663,  680,  698,  715,  732,  749,  766,  783,  800,  817,  834,  851,  867,  884,  900,  916,  932, 
	949,  964,  980,  996,  1012,  1027,  1043,  1058,  1073,  1088,  1103,  1118,  1133,  1147,  1162,  1176, 
	1190,  1204,  1218,  1232,  1246,  1259,  1273,  1286,  1299,  1312,  1325,  1338,  1350,  1363,  1375,  1387, 
	1399,  1411,  1422,  1434,  1445,  1456,  1467,  1478,  1489,  1499,  1510,  1520,  1530,  1540,  1549,  1559, 
	1568,  1577,  1586,  1595,  1604,  1612,  1621,  1629,  1637,  1644,  1652,  1659,  1667,  1674,  1680,  1687, 
	1694,  1700,  1706,  1712,  1718,  1723,  1729,  1734,  1739,  1743,  1748,  1752,  1757,  1761,  1764,  1768,
  1772,  1775,  1778,  1781,  1783,  1786,  1788,  1790,  1792,  1794,  1795,  1796,  1798,  1798,  1799,  1800, 
	1800,  1800,  1800,  1800,  1799,  1798,  1798,  1796,  1795,  1794,  1792,  1790,  1788,  1786,  1783,  1781, 
	1778,  1775,  1772,  1768,  1764,  1761,  1757,  1752,  1748,  1743,  1739,  1734,  1729,  1723,  1718,  1712,
  1706,  1700,  1694,  1687,  1680,  1674,  1667,  1659,  1652,  1644,  1637,  1629,  1621,  1612,  1604,  1595, 
	1586,  1577,  1568,  1559,  1549,  1540,  1530,  1520,  1510,  1499,  1489,  1478,  1467,  1456,  1445,  1434, 
	1422,  1411,  1399,  1387,  1375,  1363,  1350,  1338,  1325,  1312,  1299,  1286,  1273,  1259,  1246,  1232,
  1218,  1204,  1190,  1176,  1162,  1147,  1133,  1118,  1103,  1088,  1073,  1058,  1043,  1027,  1012,  996, 
	980,  964,  949,  932,  916,  900,  884,  867,  851,  834,  817,  800,  783,  766,  749,  732,  715,  698, 
	680,  663,  645,  627,  610,  592,  574,  556,  538,  520,  502,  484,  466,  448,  429,  411,  393,  374,  
	356,  337,  319,  300,  282,  263,  244,  226,  207,  188,  169,  151,  132,  113,  94,  75,  57,  38,  19, 
	0,  -19,  -38,  -57,  -75,  -94,  -113,  -132,  -151,  -169,  -188,  -207,  -226,  -244,  -263,  -282,  -300, 
	-319,  -337,  -356,  -374,  -393,  -411,  -429,  -448,  -466,  -484,  -502,  -520,  -538,  -556,  -574,  -592,
  -610,  -627,  -645,  -663,  -680,  -698,  -715,  -732,  -749,  -766,  -783,  -800,  -817,  -834,  -851,  -867,
  -884,  -900,  -916,  -932,  -949,  -964,  -980,  -996,  -1012,  -1027,  -1043,  -1058,  -1073,  -1088,  -1103,
  -1118,  -1133,  -1147,  -1162,  -1176,  -1190,  -1204,  -1218,  -1232,  -1246,  -1259,  -1273,  -1286,  -1299,
  -1312,  -1325,  -1338,  -1350,  -1363,  -1375,  -1387,  -1399,  -1411,  -1422,  -1434,  -1445,  -1456,  -1467,
  -1478,  -1489,  -1499,  -1510,  -1520,  -1530,  -1540,  -1549,  -1559,  -1568,  -1577,  -1586,  -1595,  -1604, 
	-1612,  -1621,  -1629,  -1637,  -1644,  -1652,  -1659,  -1667,  -1674,  -1680,  -1687,  -1694,  -1700,  -1706, 
	-1712,  -1718,  -1723,  -1729,  -1734,  -1739,  -1743,  -1748,  -1752,  -1757,  -1761,  -1764,  -1768,  -1772, 
	-1775,  -1778,  -1781,  -1783,  -1786,  -1788,  -1790,  -1792,  -1794,  -1795,  -1796,  -1798,  -1798,  -1799, 
	-1800,  -1800,  -1800,  -1800,  -1800,  -1799,  -1798,  -1798,  -1796,  -1795,  -1794,  -1792,  -1790,  -1788,
  -1786,  -1783,  -1781,  -1778,  -1775,  -1772,  -1768,  -1764,  -1761,  -1757,  -1752,  -1748,  -1743,  -1739,
  -1734,  -1729,  -1723,  -1718,  -1712,  -1706,  -1700,  -1694,  -1687,  -1680,  -1674,  -1667,  -1659,  -1652,
  -1644,  -1637,  -1629,  -1621,  -1612,  -1604,  -1595,  -1586,  -1577,  -1568,  -1559,  -1549,  -1540,  -1530,
  -1520,  -1510,  -1499,  -1489,  -1478,  -1467,  -1456,  -1445,  -1434,  -1422,  -1411,  -1399,  -1387,  -1375,
  -1363,  -1350,  -1338,  -1325,  -1312,  -1299,  -1286,  -1273,  -1259,  -1246,  -1232,  -1218,  -1204,  -1190,
  -1176,  -1162,  -1147,  -1133,  -1118,  -1103,  -1088,  -1073,  -1058,  -1043,  -1027,  -1012,  -996,  -980,  
	-964,  -949,  -932,  -916,  -900,  -884,  -867,  -851,  -834,  -817,  -800,  -783,  -766,  -749,  -732,  -715,
  -698,  -680,  -663,  -645,  -627,  -610,  -592,  -574,  -556,  -538,  -520,  -502,  -484,  -466,  -448,  -429,
  -411,  -393,  -374,  -356,  -337,  -319,  -300,  -282,  -263,  -244,  -226,  -207,  -188,  -169,  -151,  -132,
-113,  -94,  -75,  -57,  -38,  -19};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Step1(void);
void Step2(void);
void Step3(void);
void Step4(void);
void Step5(void);
void Step6(void);
void SW1_PWM_SW4_off(void);
void SW2_PWM_SW5_off(void);
void SW3_PWM_SW6_off(void);
void SW4_PWM_SW1_off(void);
void SW5_PWM_SW2_off(void);
void SW6_PWM_SW3_off(void);
void DisableAll(void);
void Step1(void)
{
	SW1_PWM_SW4_off();
	SW3_PWM_SW6_off();
	SW5_PWM_SW2_off();
	TIM1->CCR1 = VREF[i];
	TIM1->CCR2 = -VREF[j];
	TIM1->CCR3 = VREF[k];
}
void Step2(void)
{
	SW1_PWM_SW4_off();
	SW5_PWM_SW2_off();
	SW6_PWM_SW3_off();
	TIM1->CCR1 = VREF[i];
	TIM1->CCR2 = -VREF[j];
	TIM1->CCR3 = -VREF[k];
}
void Step3(void)
{
	SW1_PWM_SW4_off();
	SW2_PWM_SW5_off();
	SW6_PWM_SW3_off();
	TIM1->CCR1 = VREF[i];
	TIM1->CCR2 = VREF[j];
	TIM1->CCR3 = -VREF[k];
}
void Step4(void)
{
	SW2_PWM_SW5_off();
	SW4_PWM_SW1_off();
	SW6_PWM_SW3_off();
	TIM1->CCR1 = -VREF[i];
	TIM1->CCR2 = VREF[j];
	TIM1->CCR3 = -VREF[k];
}
void Step5(void)
{
	SW2_PWM_SW5_off();
	SW3_PWM_SW6_off();
	SW4_PWM_SW1_off();
	TIM1->CCR1 = -VREF[i];
	TIM1->CCR2 = VREF[j];
	TIM1->CCR3 = VREF[k];
}
void Step6(void)
{
	SW3_PWM_SW6_off();
	SW5_PWM_SW2_off();
	SW4_PWM_SW1_off();
	TIM1->CCR1 = -VREF[i];
	TIM1->CCR2 = -VREF[j];
	TIM1->CCR3 = VREF[k];
}
void SW1_PWM_SW4_off(void)
{
	//turn off SW4 
	SW4_A7_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_SET);
	//turn on PWM on PA8 (TIM1_CH1)
	SW1_A8_PWM_MODE_CH1;
}
void SW2_PWM_SW5_off(void)
{
	//turn off SW5
	SW5_B0_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_SET);
 	//turn on PWM on PA9 (TIM1_CH2)
	SW2_A9_PWM_MODE_CH2;
}
void SW3_PWM_SW6_off(void)
{
		//turn off SW5
	SW6_B1_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET);
 	//turn on PWM on PA10 (TIM1_CH3)
	SW3_A10_PWM_MODE_CH3;
}
void SW4_PWM_SW1_off(void)
{
	//turn off SW1
	SW1_A8_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	//turn on PWM on PA7 (TIM1_CH1)
	SW4_A7_PWM_MODE_CH1N;
}
void SW5_PWM_SW2_off(void)
{
	//turn off SW2
	SW2_A9_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	//turn on PWM on PB0 (TIM1_CH2)
	SW5_B0_PWM_MODE_CH2N;
}
void SW6_PWM_SW3_off(void)
{
	//turn off SW3
	SW3_A10_GPIO_MODE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	//turn on PWM on PB1 (TIM1_CH3)
	SW6_B1_PWM_MODE_CH3N;
}

void DisableAll(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_SET);
	SW1_A8_GPIO_MODE;
	SW2_A9_GPIO_MODE;
	SW3_A10_GPIO_MODE;
	SW4_A7_GPIO_MODE;
	SW5_B0_GPIO_MODE;
	SW6_B1_GPIO_MODE;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ((htim->Instance == TIM1)&(TIM1->CNT < 1600))
	{
		if (i<WIDTH_OF_STEP)
		{
			Step1();
		}
		else if (i<(2*WIDTH_OF_STEP))
		{
			Step2();
		}
		else if (i<(3*WIDTH_OF_STEP))
		{
			Step3();
		}
		else if (i<(4*WIDTH_OF_STEP))
		{
			Step4();
		}
		else if (i<(5*WIDTH_OF_STEP))
		{
			Step5();
		}
		else if (i<(6*WIDTH_OF_STEP))
		{
			Step6();
		}
		temp = VREF[i];
		HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		i++;	
		j++;
		k++;
		if (i == PULSE_PER_CYCLE) i = 0;
		if (j == PULSE_PER_CYCLE) j = 0;
		if (k == PULSE_PER_CYCLE) k = 0;
		HAL_GPIO_TogglePin(LED_KIT_GPIO_Port, LED_KIT_Pin);
	}
	if (htim->Instance == TIM3)
	{
	}	
	if (htim->Instance == TIM2)
	{
		
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	DisableAll();
  /* USER CODE END 2 */

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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_KIT_GPIO_Port, LED_KIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_KIT_Pin */
  GPIO_InitStruct.Pin = LED_KIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_KIT_GPIO_Port, &GPIO_InitStruct);

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
