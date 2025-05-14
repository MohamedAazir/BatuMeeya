#include "main.h"
#include "vl53l0x_api.h"
#include <stdio.h>

// Define the I2C handle
I2C_HandleTypeDef hi2c1;



// Define new I2C addresses for the sensors
#define LOX1_ADDRESS 0x52


// Global variables to store distance measurements
uint16_t tof = 0;

uint16_t error = 0;


// Declare sensor devices
VL53L0X_Dev_t sensor;


void SystemClock_Config(void);
static void MX_I2C1_Init(void);
void VL53L0X_Init_Sensor(VL53L0X_Dev_t *sensor, uint8_t new_address);
void read_three_sensors(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();

    // Initialize sensors one by one
    VL53L0X_Init_Sensor(&sensor, LOX1_ADDRESS);


    while (1) {
        read_three_sensors();
        printf("Sensor 1: %d mm, Sensor 2: %d mm, Sensor 3: %d mm\n", tof);
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
    if (VL53L0X_PerformSingleRangingMeasurement(&sensor, &RangingData) == VL53L0X_ERROR_NONE) {
        tof = RangingData.RangeMilliMeter;
    } else {
        printf("Error reading sensor 1\n");
        error = 8;
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
