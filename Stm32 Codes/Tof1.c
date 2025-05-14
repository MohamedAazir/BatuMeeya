#include "main.h"
#include "vl53l0x_api.h"

// Define the I2C handle
I2C_HandleTypeDef hi2c1;

// Define GPIO pins for XSHUT control
#define XSHUT_PIN GPIO_PIN_0
#define XSHUT_PORT GPIOA

// Define GPIO1 pin
#define GPIO1_PIN GPIO_PIN_3
#define GPIO1_PORT GPIOB

// Define new I2C address for the sensor
#define SENSOR_NEW_ADDR 0x30

// Global variable to store distance measurement
uint16_t sensor_distance = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void VL53L0X_Init_Sensor(VL53L0X_Dev_t *sensor, uint8_t new_address);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Define sensor device
    VL53L0X_Dev_t sensor;

    // Initialize sensor
    VL53L0X_Init_Sensor(&sensor, SENSOR_NEW_ADDR);

    while (1) {
        VL53L0X_RangingMeasurementData_t RangingData;

        // Read distance from sensor
        VL53L0X_PerformSingleRangingMeasurement(&sensor, &RangingData);
        sensor_distance = RangingData.RangeMilliMeter;

        HAL_Delay(50);
    }
}

void VL53L0X_Init_Sensor(VL53L0X_Dev_t *sensor, uint8_t new_address) {
    sensor->I2cHandle = &hi2c1;
    sensor->I2cDevAddr = 0x52; // Default I2C address

    // Initialize the sensor
    VL53L0X_DataInit(sensor);
    VL53L0X_StaticInit(sensor);

    // Change the I2C address
    VL53L0X_SetDeviceAddress(sensor, new_address);
    sensor->I2cDevAddr = new_address;

    // Declare necessary variables for calibration and SPAD management
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;
    uint32_t refSpadCount = 0;
    uint8_t isApertureSpads = 0;

    // Perform sensor calibration and initialization
    VL53L0X_PerformRefCalibration(sensor, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(sensor, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(sensor, VL53L0X_DEVICEMODE_SINGLE_RANGING);
}

void SystemClock_Config(void) {
    // System clock configuration code
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

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

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin for XSHUT control
    GPIO_InitStruct.Pin = XSHUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(XSHUT_PORT, &GPIO_InitStruct);

    // Set XSHUT pin to high to enable sensor
    HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_PIN, GPIO_PIN_SET);

    // Configure GPIO pin for GPIO1
    GPIO_InitStruct.Pin = GPIO1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO1_PORT, &GPIO_InitStruct);

    // Delay to ensure sensor is powered up
    HAL_Delay(100);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    // Configure I2C SCL and SDA pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
    // Error handling code
    while (1) {
        // Stay here
    }
}
