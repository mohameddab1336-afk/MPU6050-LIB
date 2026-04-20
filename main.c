/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - MPU6050 + Madgwick AHRS Filter
  * @author         : Drone Project
  * @date           : 2026
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes */
#include "main.h"

/* Private includes */
/* USER CODE BEGIN Includes */
#include "TJ_MPU6050.h"
#include "Madgwick_AHRS.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef */
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define */
/* USER CODE BEGIN PD */
#define SAMPLE_TIME    0.005f   /* 5 ms sample period (200 Hz) */
#define MADGWICK_BETA  0.1f     /* Filter convergence rate      */
/* USER CODE END PD */

/* Private macro */
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables */
I2C_HandleTypeDef  hi2c1;
I2S_HandleTypeDef  hi2s3;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Sensor data */
ScaledData_Def accel;
ScaledData_Def gyro;

/* Euler angles returned by the Madgwick filter */
EulerAngles_t angles;

/* Loop counter used to throttle UART output */
uint32_t loopCounter = 0;
/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    MPU_ConfigTypeDef myMpuConfig;
    /* USER CODE END 1 */

    /* MCU Configuration */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    /* ===== INITIALIZE MPU6050 ===== */
    MPU6050_Init(&hi2c1);

    myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;         /* 4 g full scale          */
    myMpuConfig.ClockSource      = Internal_8MHz;       /* Internal 8 MHz clock    */
    myMpuConfig.CONFIG_DLPF      = DLPF_184A_188G_Hz;  /* 184 Hz accel / 188 Hz gyro DLPF */
    myMpuConfig.Gyro_Full_Scale  = FS_SEL_500;          /* 500 deg/sec full scale  */
    myMpuConfig.Sleep_Mode_Bit   = 0;                   /* Wake up (no sleep)      */
    MPU6050_Config(&myMpuConfig);

    /* ===== INITIALIZE MADGWICK FILTER ===== */
    /* Beta = 0.1 (optimal for drone applications), 200 Hz sample rate */
    Madgwick_Init(MADGWICK_BETA, 200.0f);

    printf("=====================================\r\n");
    printf("  MPU6050 + Madgwick AHRS Filter\r\n");
    printf("=====================================\r\n");
    printf("   Roll  |  Pitch  |   Yaw\r\n");
    printf("=====================================\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* ===== READ SENSOR DATA ===== */
        MPU6050_Get_Accel_Scale(&accel);   /* Accelerometer in g          */
        MPU6050_Get_Gyro_Scale(&gyro);     /* Gyroscope    in deg/sec     */

        /* ===== UPDATE MADGWICK FILTER ===== */
        /* Fixed 5 ms time step matching the 200 Hz HAL_Delay below */
        Madgwick_UpdateIMU(gyro.x,  gyro.y,  gyro.z,
                           accel.x, accel.y, accel.z,
                           SAMPLE_TIME);

        /* ===== GET ROLL, PITCH, YAW ===== */
        angles = Madgwick_GetEulerAngles();

        /* ===== DISPLAY RESULTS every 10 loops (~50 ms) ===== */
        if (loopCounter++ % 10 == 0)
        {
            printf("Roll: %6.2f  Pitch: %6.2f  Yaw: %6.2f\r\n",
                   (double)angles.roll,
                   (double)angles.pitch,
                   (double)angles.yaw);
        }

        /* ===== DRONE STABILIZATION LOGIC (example placeholders) ===== */
        if (angles.pitch > 30.0f)
        {
            /* Drone tilts forward — reduce front motors, increase rear motors */
        }

        if (angles.roll > 30.0f)
        {
            /* Drone tilts right — reduce right motors, increase left motors */
        }

        /* 5 ms delay → 200 Hz loop */
        HAL_Delay(5);

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief  System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  I2C1 Initialization
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  GPIO Initialization
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

/* Peripheral init stubs — fill in as needed for your board */
static void MX_I2S3_Init(void)   { }
static void MX_SPI1_Init(void)   { }
static void MX_TIM1_Init(void)   { }
static void MX_USART2_UART_Init(void) { }

/**
  * @brief  Error handler — disable interrupts and halt.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
