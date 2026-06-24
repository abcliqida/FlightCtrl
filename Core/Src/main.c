/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
}imu_data_t;

typedef struct {
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
}mag_data_t;

#pragma pack(push, 1)   // 关键：1 字节对齐，禁止编译器插 padding

typedef struct
{
  uint32_t iTOW;      // [ms] GPS 时间周内秒
  uint16_t year;      // 年，如 2026
  uint8_t  month;     // 月 1..12
  uint8_t  day;       // 日 1..31
  uint8_t  hour;      // 时 0..23
  uint8_t  min;       // 分 0..59
  uint8_t  sec;       // 秒 0..60
  uint8_t  valid;     // 有效性标志
  uint32_t tAcc;      // [ns] 时间精度

  int32_t  nano;      // [ns] 纳秒部分（-1e9..+1e9）

  uint8_t  fixType;   // 定位类型：0=无，3=3D，4=GNSS+DR
  uint8_t  flags;     // 定位标志
  uint8_t  flags2;
  uint8_t  numSV;     // 使用卫星数

  int32_t  lon;       // [1e-7 °] 经度
  int32_t  lat;       // [1e-7 °] 纬度
  int32_t  height;    // [mm] 椭球高
  int32_t  hMSL;      // [mm] 海拔高

  uint32_t hAcc;      // [mm] 水平精度
  uint32_t vAcc;      // [mm] 垂直精度

  int32_t  velN;      // [mm/s] 北向速度
  int32_t  velE;      // [mm/s] 东向速度
  int32_t  velD;      // [mm/s] 地向速度（向下为正）
  int32_t  gSpeed;    // [mm/s] 地面速率

  int32_t  headMot;   // [1e-5 °] 运动航向
  uint32_t sAcc;      // [mm/s] 速度精度

  uint32_t headAcc;   // [1e-5 °] 航向精度
  uint16_t pDOP;      // [0.01] 位置 DOP

  uint8_t  reserved1[6];

  int32_t  headVeh;   // [1e-5 °] 车头方向（仅车载模式）
  int16_t  magDec;    // [0.01 °] 磁偏角
  uint16_t magAcc;    // [0.01 °] 磁偏角精度
} UBX_NAV_PVT_t;

#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ICM45686_CS_PORT GPIOA
#define ICM45686_CS_PIN GPIO_PIN_4
#define ICM_REG_WHO_AM_I        0x72                          //dv
#define ICM_WHOAMI_VALUE        0xE9                          //dv
#define ICM_REG_MISC2           0x7F                          //dv
#define ICM_RESET               0x02                          //???
#define ICM_REG_ACCEL_CONFIG    0x1B                          //dv
#define ICM_REG_PWR_MGMT0       0x10                          //dv
#define ICM_REG_GYRO_CONFIG0    0x1C                          //dv
#define ICM_REG_ACCEL_X_UPPER   0x00                          //dv
#define ICM_SREG_CTRL           0x67                          //dv


#define GYRO_UI_FS_SEL_4000DPS  0x00                          //dv
#define GYRO_UI_FS_SEL_2000DPS  0x01                          //dv
#define GYRO_UI_FS_SEL_1000DPS  0x02                          //dv
#define GYRO_UI_FS_SEL_500DPS   0x03                          //dv
#define GYRO_UI_FS_SEL_250DPS   0x04                          //dv
#define GYRO_UI_FS_SEL_125DPS   0x05                          //dv
#define GYRO_ODR_6400Hz         0x03                          //dv
#define GYRO_ODR_3200Hz         0x04                          //dv
#define GYRO_ODR_1600Hz         0x05                          //dv
#define GYRO_ODR_800Hz          0x06                          //dv
#define GYRO_ODR_400Hz          0x07                          //dv
#define GYRO_ODR_200Hz          0x08                          //dv
#define GYRO_ODR_100Hz          0x09                          //dv
#define GYRO_ODR_50Hz           0x0A                          //dv
#define GYRO_ODR_25Hz           0x0B                          //dv
#define GYRO_ODR_12_5Hz         0x0C                          //dv
#define GYRO_ODR_6_25Hz         0x0D                          //dv

#define ACCEL_UI_FS_SEL_32G     0x00                          //dv
#define ACCEL_UI_FS_SEL_16G     0x01                          //dv
#define ACCEL_UI_FS_SEL_8G      0x02                          //dv
#define ACCEL_UI_FS_SEL_4G      0x03                          //dv
#define ACCEL_UI_FS_SEL_2G      0x04                          //dv
#define ACCEL_ODR_6400Hz        0x03                          //dv
#define ACCEL_ODR_3200Hz        0x04                          //dv
#define ACCEL_ODR_1600Hz        0x05                          //dv
#define ACCEL_ODR_800Hz         0x06                          //dv
#define ACCEL_ODR_400Hz         0x07                          //dv
#define ACCEL_ODR_200Hz         0x08                          //dv
#define ACCEL_ODR_100Hz         0x09                          //dv
#define ACCEL_ODR_50Hz          0x0A                          //dv
#define ACCEL_ODR_25Hz          0x0B                          //dv
#define ACCEL_ODR_12_5Hz        0x0C                          //dv

#define QMC_CHIPID_ADDR         0x00                          //dv
#define QMC_CHIPID              0x80                          //dv
#define QMC_DEV_ADDR            0x2c                          //dv
#define QMC_REG_XOUT_L          0x01                          //dv
#define QMC_REG_STATUS          0x09                          //dv
#define QMC_REG_CTRL1           0x0A                          //dv
#define QMC_REG_CTRL2           0x0B                          //dv
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
imu_data_t imu_data = {0};
uint8_t uart4_rx_data[512] __attribute__((section(".dma_buffer")));
UBX_NAV_PVT_t pvt = {0};
uint8_t i2c1_rx_data[6] = {0};
mag_data_t mag_data = {0};
uint8_t qmc_status = {0};
uint8_t qmc_init_ret = 0xFF;
uint8_t qmc_read_ret1 = 0xFF;
uint8_t qmc_read_ret2 = 0xFF;
uint8_t temp2 = 0;
uint8_t temp3 = 0;
uint8_t temp4 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
uint8_t ICM45686_ReadReg(uint8_t addr, uint8_t *val);
uint8_t ICM45686_WriteReg(uint8_t addr, uint8_t val);
uint8_t ICM45686_Init(void);
uint8_t QMC5883_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  ICM45686_Init();
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim6);
  qmc_init_ret = QMC5883_Init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_rx_data,512);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(100);
    qmc_read_ret1 = HAL_I2C_Mem_Read(&hi2c1,QMC_DEV_ADDR<<1,QMC_REG_XOUT_L,I2C_MEMADD_SIZE_8BIT,i2c1_rx_data,6,HAL_MAX_DELAY);
    mag_data.mag_x = (int16_t)(i2c1_rx_data[1]<<8 | i2c1_rx_data[0]);
    mag_data.mag_y = (int16_t)(i2c1_rx_data[3]<<8 | i2c1_rx_data[2]);
    mag_data.mag_z = (int16_t)(i2c1_rx_data[5]<<8 | i2c1_rx_data[4]);
    qmc_read_ret2 = HAL_I2C_Mem_Read(&hi2c1,QMC_DEV_ADDR<<1,QMC_REG_STATUS,I2C_MEMADD_SIZE_8BIT,&qmc_status,1,HAL_MAX_DELAY);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

    ICM45686_ReadReg(ICM_REG_PWR_MGMT0,&temp2);
    ICM45686_ReadReg(ICM_REG_ACCEL_CONFIG,&temp3);
    ICM45686_ReadReg(ICM_REG_GYRO_CONFIG0,&temp4);


    /// 读取IMU数据
    uint8_t tx_data[13] = {0};
    uint8_t rx_data[13] = {0};

    tx_data[0] = ICM_REG_ACCEL_X_UPPER | 0x80;
    HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 13, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_SET);
    imu_data.accel_x = (int16_t)(rx_data[2]<<8 | rx_data[1])/4096.f;
    imu_data.accel_y = (int16_t)(rx_data[4]<<8 | rx_data[3])/4096.f;
    imu_data.accel_z = (int16_t)(rx_data[6]<<8 | rx_data[5])/4096.f;
    imu_data.gyro_x = (int16_t)(rx_data[8]<<8 | rx_data[7])/32768.f*2000.f/180.f*M_PI;
    imu_data.gyro_y = (int16_t)(rx_data[10]<<8 | rx_data[9])/32768.f*2000.f/180.f*M_PI;
    imu_data.gyro_z = (int16_t)(rx_data[12]<<8 | rx_data[11])/32768.f*2000.f/180.f*M_PI;
  }
}

uint8_t ICM45686_Init(void)
{
  uint8_t reg_val = 0;                                                                             //dv
  if (ICM45686_ReadReg(ICM_REG_WHO_AM_I, &reg_val) != 0 || reg_val != ICM_WHOAMI_VALUE)       //dv
    return 1;                                                                                      //dv
  ICM45686_WriteReg(ICM_REG_MISC2, ICM_RESET);                                            //dv
  do {                                                                                             //dv
    if (ICM45686_ReadReg(ICM_REG_MISC2, &reg_val) != 0)                                       //dv
      return 1;                                                                                    //dv
  } while (reg_val & 0x02);                                                                        //dv
  // ICM45686_WriteReg(ICM_REG_PWR_MGMT_1, ICM_CLOCK_AUTO);                                        //???
  HAL_Delay(30);

  ICM45686_WriteReg(ICM_REG_PWR_MGMT0,0x0F);
  HAL_Delay(30);

  ICM45686_WriteReg(ICM_REG_ACCEL_CONFIG, (uint8_t)(ACCEL_UI_FS_SEL_8G << 4 | ACCEL_ODR_6400Hz));
  HAL_Delay(30);

  ICM45686_WriteReg(ICM_REG_GYRO_CONFIG0,(uint8_t)(GYRO_UI_FS_SEL_2000DPS << 4 | GYRO_ODR_6400Hz));

  return 0;                                                                                        //dv
}

uint8_t ICM45686_ReadReg(uint8_t addr, uint8_t *val)  //dv
{
  uint8_t tx_buf[2];                                  //dv
  uint8_t rx_buf[2];                                  //dv
  tx_buf[0] = addr | 0x80;                            //dv
  tx_buf[1] = 0x00;                                   //dv
  HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_RESET);                       //dv
  if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK)      //dv
  {
    HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_SET);                       //dv
    return 1;                                                                                       //dv
  }
  HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_SET);                         //dv
  *val = rx_buf[1];                                                                                 //dv
  return 0;                                                                                         //dv
}


uint8_t ICM45686_WriteReg(uint8_t addr, uint8_t val)                              //dv
{
  uint8_t tx_buf[2] = { addr & 0x7F, val };                                       //dv
  uint8_t rx_buf[2];                                                              //dv

  HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_RESET);     //dv
  if(HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_SET);       //dv
    return 1;
  }
  HAL_GPIO_WritePin(ICM45686_CS_PORT,ICM45686_CS_PIN,GPIO_PIN_SET);       //dv
  return 0;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance == UART4)
  {
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
    if (uart4_rx_data[0] == 0xB5 && uart4_rx_data[1] == 0x62)
    {
      if (uart4_rx_data[2] == 0x01 && uart4_rx_data[3] == 0x07)
      {
        memcpy(&pvt, &uart4_rx_data[6], sizeof(UBX_NAV_PVT_t));
      }
    }
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_rx_data,512);
  }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == UART4)
  {
    // uint32_t err = HAL_UART_GetError(huart);

    HAL_UART_DMAStop(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    // 复位 HAL 库内部的串口状态（非常重要！否则库会一直认为串口处于 BUSY 状态）
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_rx_data,512);
  }
}


uint8_t QMC5883_Init(void)
{
  uint8_t reg_val;
  if ( HAL_I2C_Mem_Read(&hi2c1,QMC_DEV_ADDR<<1,QMC_CHIPID_ADDR,\
    I2C_MEMADD_SIZE_8BIT,&reg_val,1,HAL_MAX_DELAY)!= HAL_OK)
  {
    return 1;
  }
  else
  {
    if (reg_val != QMC_CHIPID)
    {
      return 2;
    }
    else
    {
      reg_val = 0x0C;
      if (HAL_I2C_Mem_Write(&hi2c1,QMC_DEV_ADDR<<1,QMC_REG_CTRL2,\
        I2C_MEMADD_SIZE_8BIT,&reg_val,1,HAL_MAX_DELAY)!= HAL_OK)
      {
        return 3;
      }

      // reg_val = 0x3F;
      reg_val = 0x33;
      if (HAL_I2C_Mem_Write(&hi2c1,QMC_DEV_ADDR<<1,QMC_REG_CTRL1,\
        I2C_MEMADD_SIZE_8BIT,&reg_val,1,HAL_MAX_DELAY)!= HAL_OK)
      {
        return 4;
      }
      return 0;
    }
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // __disable_irq();
  // while (1)
  // {
  // }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
