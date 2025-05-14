#include "main.h"
#include "vl53l0x_api.h"
#include <stdio.h>

// Define the I2C handle
I2C_HandleTypeDef hi2c1;

// Define GPIO pins for XSHUT control
#define XSHUT1_PIN GPIO_PIN_0
#define XSHUT1_PORT GPIOA
#define XSHUT2_PIN GPIO_PIN_1
#define XSHUT2_PORT GPIOA
#define XSHUT3_PIN GPIO_PIN_2
#define XSHUT3_PORT GPIOA


// Define new I2C addresses for the sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// Global variables to store distance measurements
uint16_t tof1 = 0;
uint16_t tof2 = 0;
uint16_t tof3 = 0;
uint16_t error = 0;


// Declare sensor devices
VL53L0X_Dev_t sensor1;
VL53L0X_Dev_t sensor2;
VL53L0X_Dev_t sensor3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void VL53L0X_Init_Sensor(VL53L0X_Dev_t *sensor, uint8_t new_address);
void read_three_sensors(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Initialize sensors one by one
    HAL_GPIO_WritePin(XSHUT1_PORT, XSHUT1_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init_Sensor(&sensor1, LOX1_ADDRESS);

    HAL_GPIO_WritePin(XSHUT2_PORT, XSHUT2_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init_Sensor(&sensor2, LOX2_ADDRESS);

    HAL_GPIO_WritePin(XSHUT3_PORT, XSHUT3_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init_Sensor(&sensor3, LOX3_ADDRESS);

    while (1) {
        read_three_sensors();
        printf("Sensor 1: %d mm, Sensor 2: %d mm, Sensor 3: %d mm\n", tof1, tof2, tof3);
        error = 1;
        HAL_Delay(100);
    }
}

void VL53L0X_Init_Sensor(VL53L0X_Dev_t *sensor, uint8_t new_address) {
    sensor->I2cHandle = &hi2c1;
    sensor->I2cDevAddr = 0x52; // Default I2C address

    // Initialize the sensor
    if (VL53L0X_DataInit(sensor) != VL53L0X_ERROR_NONE) {
        printf("Error initializing sensor\n");
        error = 2;
        return;
    }
    if (VL53L0X_StaticInit(sensor) != VL53L0X_ERROR_NONE) {
        printf("Error in static initialization\n");
        error = 3;
        return;
    }

    // Change the I2C address
    if (VL53L0X_SetDeviceAddress(sensor, new_address) != VL53L0X_ERROR_NONE) {
        printf("Error setting device address\n");
        error = 4;
        return;
    }
    sensor->I2cDevAddr = new_address;

    // Declare necessary variables for calibration and SPAD management
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;
    uint32_t refSpadCount = 0;
    uint8_t isApertureSpads = 0;

    // Perform sensor calibration and initialization
    if (VL53L0X_PerformRefCalibration(sensor, &VhvSettings, &PhaseCal) != VL53L0X_ERROR_NONE) {
        printf("Error in reference calibration\n");
        error = 5;
        return;
    }
    if (VL53L0X_PerformRefSpadManagement(sensor, &refSpadCount, &isApertureSpads) != VL53L0X_ERROR_NONE) {
        printf("Error in SPAD management\n");
        error = 6;
        return;
    }
    if (VL53L0X_SetDeviceMode(sensor, VL53L0X_DEVICEMODE_SINGLE_RANGING) != VL53L0X_ERROR_NONE) {
        printf("Error setting device mode\n");
        error = 7;
        return;
    }
}

void read_three_sensors(void) {
    VL53L0X_RangingMeasurementData_t RangingData;

    // Read distance from sensor 1
    if (VL53L0X_PerformSingleRangingMeasurement(&sensor1, &RangingData) == VL53L0X_ERROR_NONE) {
        tof1 = RangingData.RangeMilliMeter;
    } else {
        printf("Error reading sensor 1\n");
        error = 8;
    }

    // Read distance from sensor 2
    if (VL53L0X_PerformSingleRangingMeasurement(&sensor2, &RangingData) == VL53L0X_ERROR_NONE) {
        tof2 = RangingData.RangeMilliMeter;
    } else {
        printf("Error reading sensor 2\n");
        error = 9;
    }

    // Read distance from sensor 3
    if (VL53L0X_PerformSingleRangingMeasurement(&sensor3, &RangingData) == VL53L0X_ERROR_NONE) {
        tof3 = RangingData.RangeMilliMeter;
    } else {
        printf("Error reading sensor 3\n");
        error = 10;
    }
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

    // Configure GPIO pins for XSHUT control
    GPIO_InitStruct.Pin = XSHUT1_PIN | XSHUT2_PIN | XSHUT3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Set XSHUT pins to low to disable sensors initially
    HAL_GPIO_WritePin(XSHUT1_PORT, XSHUT1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT2_PORT, XSHUT2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT3_PORT, XSHUT3_PIN, GPIO_PIN_RESET);



    // Delay to ensure sensors are powered up
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
