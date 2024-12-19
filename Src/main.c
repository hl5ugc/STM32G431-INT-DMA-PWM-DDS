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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_RESOLUTION  1024
#define NS              128
#define MAIN_CLK        170000000
#define F_SIGNAL        100
#define TIM4CLK_DIV     1
#define TIM4CLK         (MAIN_CLK / TIM4CLK_DIV)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint32_t Wave_LUT[NS] = {
    512, 537, 562, 587, 612, 637, 661, 685, 709, 732, 754, 776, 798, 818, 838,
    857, 875, 893, 909, 925, 939, 952, 965, 976, 986, 995, 1002,1009,1014,1018,
    1021,1023,1023,1022,1020,1016,1012,1006,999, 990, 981, 970, 959, 946, 932,
    917, 901, 884, 866, 848, 828, 808, 787, 765, 743, 720, 697, 673, 649, 624,
    600, 575, 549, 524, 499, 474, 448, 423, 399, 374, 350, 326, 303, 280, 258,
    236, 215, 195, 175, 157, 139, 122, 106, 91,  77,  64,  53,  42,  33,  24,
    17,  11,  7,   3,   1,   0,   0,   2,   5,   9,   14,  21,  28,  37,  47,
    58,  71,  84,  98,  114, 130, 148, 166, 185, 205, 225, 247, 269, 291, 314,
    338, 362, 386, 411, 436, 461, 486, 511
};
static uint32_t Wave_LUT1[NS] ;

static uint32_t Saw_LUT[NS]  ;
uint32_t DestAddress  = (uint32_t)&(TIM3->CCR1) ;
uint32_t TIM4_Ticks   = (TIM4CLK / (NS * F_SIGNAL) - 1) ;
__IO static float volt = 1.0f ;
__IO static uint16_t voltUpDown = 0U ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void TransferComplete(DMA_HandleTypeDef *DmaHandle);
static void TransferError(DMA_HandleTypeDef *DmaHandle);
static void TransferHalfComplete(DMA_HandleTypeDef *DmaHandle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Saw_LUT_Gen(void)
{
  uint16_t i = 0U ;

  for(i=0U; i<NS; i++)
  {
    Saw_LUT[i] = (PWM_RESOLUTION/NS) * i ;

    Wave_LUT1[i] = Wave_LUT[i] * (1.0/3.2);
  }
}
static uint32_t dma_count = 0U ;
/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
  uint16_t  i = 0U ;

  if(DmaHandle->Instance == DMA1_Channel1 )
  {
    for(i=NS/2 ; i<NS; i++)
    {
      Wave_LUT1[i] = Wave_LUT[i] *  (volt / 3.2f);
    }
    if(voltUpDown == 0U)
    {
      volt = (volt + 0.1f);
      if(volt > 3.1f) { voltUpDown = 1U; }
    }
    else
    {
      volt = (volt - 0.1f);
      if(volt < 1.0f) { voltUpDown = 0U; }
    }

  }
}

static void TransferHalfComplete(DMA_HandleTypeDef *DmaHandle)
{
  uint16_t  i = 0U ;

  if(DmaHandle->Instance == DMA1_Channel1 )
  {
    for(i=0U; i<NS/2; i++)
    {
      Wave_LUT1[i] = Wave_LUT[i] *  (volt / 3.2f);
    }
  }
}
static void TransferError(DMA_HandleTypeDef *DmaHandle)
{

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Saw_LUT_Gen();

  __HAL_TIM_SET_PRESCALER(&htim4,(TIM4CLK_DIV-1));
  __HAL_TIM_SET_AUTORELOAD(&htim4,TIM4_Ticks);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
  //

  HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
  HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_HALFCPLT_CB_ID, TransferHalfComplete);
  HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_ERROR_CB_ID, TransferError);
  //
  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)Wave_LUT1, DestAddress, NS);
//  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)Saw_LUT, DestAddress, NS);
  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
