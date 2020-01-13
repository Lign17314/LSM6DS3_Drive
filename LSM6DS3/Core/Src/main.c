/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdio.h"

#include "LSM6DS3_ACC_GYRO_driver_HL.h"

#include "component.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "humidity.h"
#include "temperature.h"
//#include "pressure.h
//#include "x_nucleo_iks01a1_accelero.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define M_INT1_GPIO_PORT           GPIOD
#define M_INT1_GPIO_CLK_ENABLE()   __GPIOD_CLK_ENABLE()
#define M_INT1_GPIO_CLK_DISABLE()  __GPIOD_CLK_DISABLE()
#define M_INT1_PIN                 INT1_Pin

#define M_INT1_EXTI_IRQn           EXTI4_15_IRQn

// ready for use
#define M_INT2_GPIO_PORT           GPIOD
#define M_INT2_GPIO_CLK_ENABLE()   __GPIOD_CLK_ENABLE()
#define M_INT2_GPIO_CLK_DISABLE()  __GPIOD_CLK_DISABLE()
#define M_INT2_PIN                 INT2_Pin

#define M_INT2_EXTI_IRQn           EXTI4_15_IRQn

typedef enum
{
  ACCELERO_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  LSM6DS0_X_0,                   /* Default on board. */
  LSM6DS3_X_0,                   /* DIL24 adapter. */
  LSM303AGR_X_0,                 /* DIL24 adapter. */
  LIS2DH12_0,                    /* DIL24 adapter. */
  H3LIS331DL_0,                  /* DIL24 adapter. */
  LIS2DW12_0,                    /* DIL24 adapter. */
  LSM6DSL_X_0,                   /* DIL24 adapter. */
  TEST_X_0                       /* For sensor availability test */
} ACCELERO_ID_t;

#define ACCELERO_SENSORS_MAX_NUM 8
#define MAX_BUF_SIZE 256

#define ORIENTATION_CHANGE_INDICATION_DELAY  100  /* LED is ON for this period [ms]. */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint32_t Int_Current_Time1 = 0; /*!< Int_Current_Time1 Value */
volatile uint32_t Int_Current_Time2 = 0; /*!< Int_Current_Time2 Value */

static volatile uint8_t mems_int1_detected       = 0;
static volatile uint8_t send_orientation_request = 0;
static char dataOut[MAX_BUF_SIZE];

static void *LSM6DS3_X_0_handle = NULL;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
static ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.
static LSM6DS3_X_Data_t LSM6DS3_X_0_Data;                         // Accelerometer - sensor 1.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static DrvStatusTypeDef initializeAllSensors(void);
static DrvStatusTypeDef enableAllSensors(void);
//static void sendOrientation(void);
static DrvStatusTypeDef BSP_LSM6DS3_ACCELERO_Init(void **handle);
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle);
DrvStatusTypeDef DIL24_Sensor_IO_ITConfig(void);
DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext(void *handle, SensorIntPin_t int_pin);
DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle);
static void sendOrientation(void);
DrvStatusTypeDef BSP_ACCELERO_Get_Event_Status_Ext(void *handle, ACCELERO_Event_Status_t *status);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
	return 	HAL_I2C_Mem_Write(&hi2c2, LSM6DS3_ACC_GYRO_I2C_ADDRESS_HIGH, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, nBytesToWrite, 100);
}
uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
	return HAL_I2C_Mem_Read(&hi2c2,LSM6DS3_ACC_GYRO_I2C_ADDRESS_HIGH, ReadAddr, nBytesToRead, pBuffer, nBytesToRead, 100);
}


/*
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
/*	uint8_t state;

	DrvContextTypeDef lsm6ds3={0};
	SensorAxes_t angular_velocity={0};
	SensorAxesRaw_t angular_velocity1={0};
	uint8_t buf[100];
	lsm6ds3.who_am_i=0x69;
	//	lsm6ds3.isEnabled=1;
	lsm6ds3.pData=&buf; */

//	setbuf(stdout, NULL);//snprint 相关初始�???
  ACCELERO_Event_Status_t status;
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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SAO_GPIO_Port, SAO_Pin, SET);//     I2C
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);
 /* state=HAL_I2C_IsDeviceReady(&hi2c2, 0XD6, 5, 100);
  LSM6DS3_X_Drv.Init(&lsm6ds3);
  LSM6DS3_X_Drv.Sensor_Enable(&lsm6ds3);*/


    /* Initialize all sensors */
  initializeAllSensors();
  /* Enable all sensors */
  enableAllSensors();

  /* Enable 6D orientation */
  BSP_ACCELERO_Enable_6D_Orientation_Ext(LSM6DS3_X_0_handle, INT1_PIN);



  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	 if (mems_int1_detected != 0)
	  	    {
	  	      if (BSP_ACCELERO_Get_Event_Status_Ext(LSM6DS3_X_0_handle, &status) == COMPONENT_OK)
	  	      {
	  	        if (status.D6DOrientationStatus != 0)
	  	        {
	  	          sendOrientation();
	  	        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  	          HAL_Delay(ORIENTATION_CHANGE_INDICATION_DELAY);
	  	        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  	        }
	  	      }
	  	    mems_int1_detected = 0;
	  	    }
/*	  	  LSM6DS3_X_Drv.Get_DRDY_Status(&lsm6ds3,&state);
	  	  if(state)
	  	  {
	  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	  		  LSM6DS3_X_Drv.Get_Axes(&lsm6ds3,&angular_velocity);
	  		LSM6DS3_X_Drv.Get_AxesRaw(&lsm6ds3,&angular_velocity1);
	  		  printf("0:  %d  %d  %d  ",angular_velocity.AXIS_X,angular_velocity.AXIS_Y,angular_velocity.AXIS_Z);
	  		printf("1:  %d  %d  %d\r\n",angular_velocity1.AXIS_X,angular_velocity1.AXIS_Y,angular_velocity1.AXIS_Z);
	  	  }
	  	  HAL_Delay(200);*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|SAO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin SAO_Pin */
  GPIO_InitStruct.Pin = CS_Pin|SAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
static DrvStatusTypeDef initializeAllSensors(void)
{
	return BSP_ACCELERO_Init(LSM6DS3_X_0, &LSM6DS3_X_0_handle);
}
static DrvStatusTypeDef enableAllSensors(void)
{
  return BSP_ACCELERO_Sensor_Enable(LSM6DS3_X_0_handle);
}
DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle)
{

  *handle = NULL;

  switch (id)
  {
    default:


    case LSM6DS3_X_0:
      return BSP_LSM6DS3_ACCELERO_Init(handle);


  }

  return COMPONENT_ERROR;
}
static DrvStatusTypeDef BSP_LSM6DS3_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

/*  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }
*/
  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].who_am_i      = LSM6DS3_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].address       = LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].instance      = LSM6DS3_X_0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DS3_X_0 ];
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pVTable       = (void *)&LSM6DS3_X_Drv;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pExtVTable    = (void *)&LSM6DS3_X_ExtDrv;

  ACCELERO_Data[ LSM6DS3_X_0 ].pComponentData = (void *)&LSM6DS3_X_0_Data;
  ACCELERO_Data[ LSM6DS3_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM6DS3_X_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}
DrvStatusTypeDef DIL24_Sensor_IO_ITConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  GPIO_InitTypeDef GPIO_InitStructureInt2;

  /* Enable INT1 GPIO clock */
  M_INT1_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = M_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT1_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT1_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(M_INT1_EXTI_IRQn);

  /* Enable INT2 GPIO clock */
  M_INT2_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = M_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT2_GPIO_PORT, &GPIO_InitStructureInt2);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT2_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(M_INT2_EXTI_IRQn);

  return COMPONENT_OK;
}
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Enable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}
DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_6D_Orientation == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_6D_Orientation(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_Instance(void *handle, uint8_t *instance)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (instance == NULL)
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XL_Ext(void *handle, uint8_t *xl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (xl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_XL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_XL(ctx, xl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XH_Ext(void *handle, uint8_t *xh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (xh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_XH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_XH(ctx, xh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YL_Ext(void *handle, uint8_t *yl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (yl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_YL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_YL(ctx, yl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YH_Ext(void *handle, uint8_t *yh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (yh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_YH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_YH(ctx, yh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext(void *handle, uint8_t *zl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (zl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_ZL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_ZL(ctx, zl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext(void *handle, uint8_t *zh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (zh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_ZH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_ZH(ctx, zh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}

static void sendOrientation(void)
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  uint8_t instance;

  BSP_ACCELERO_Get_Instance(LSM6DS3_X_0_handle, &instance);

  if (BSP_ACCELERO_Get_6D_Orientation_XL_Ext(LSM6DS3_X_0_handle, &xl) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation XL axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }
  if (BSP_ACCELERO_Get_6D_Orientation_XH_Ext(LSM6DS3_X_0_handle, &xh) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation XH axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }
  if (BSP_ACCELERO_Get_6D_Orientation_YL_Ext(LSM6DS3_X_0_handle, &yl) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation YL axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }
  if (BSP_ACCELERO_Get_6D_Orientation_YH_Ext(LSM6DS3_X_0_handle, &yh) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation YH axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }
  if (BSP_ACCELERO_Get_6D_Orientation_ZL_Ext(LSM6DS3_X_0_handle, &zl) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation ZL axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }
  if (BSP_ACCELERO_Get_6D_Orientation_ZH_Ext(LSM6DS3_X_0_handle, &zh) == COMPONENT_ERROR)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "Error getting 6D orientation ZH axis from LSM6DS3 - accelerometer[%d].\r\n", instance);
    HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    return;
  }

  if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |  *             | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |________________| \r\n");
  }

  else if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |             *  | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |  *             | " \
             "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |             *  | " \
             "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  __*_____________  " \
             "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0)
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |________________| " \
             "\r\n    *               \r\n");
  }

  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "None of the 6D orientation axes is set in LSM6DS3 - accelerometer[%d].\r\n", instance);
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
}
DrvStatusTypeDef BSP_ACCELERO_Get_Event_Status_Ext(void *handle, ACCELERO_Event_Status_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Event_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Event_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  /* 6D orientation (available only for LSM6DS3 sensor). */
   if (GPIO_Pin == M_INT1_PIN)
  {
    mems_int1_detected = 1;
  }
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Falling_Callback could be implemented in the user file
   */
  if (GPIO_Pin == M_INT1_PIN)
    {
      mems_int1_detected = 1;
    }
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Falling_Callback could be implemented in the user file
   */
  if (GPIO_Pin == M_INT1_PIN)
    {
      mems_int1_detected = 1;
    }
}
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
