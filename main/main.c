/*



    ERROR CODE REFERENCE:
    1000:   CANNOT INITIALIZE I2C MASTER BUS
    1001:   CANNOT INITIALIZE I2C MPU6050 DEVICE
    1002:   CANNOT SET MPU6050 CONTINUOUS MODE THROUGH INTERNAL REGISTERS
    1003:   CANNOT SET MPU6050 GYROSCOPE SCALE FACTOR THROUGH INTERNAL REGISTERS
    1004:   CANNOT SET MPU6050 ACCELEROMETER SCALE FACTOR THROUGH INTERNAL REGISTERS
    1005:   CANNOT INITIALIZE DAC FREQUENCY CHECKER MODULE
    1006:   CANNOT SET MPU6050 LOW PASS FILTER
    1007:   CANNOT SET DEBUGGING LEDS
    1008:   CANNOT SET DEBUGGING LEDS QUEUE HANDLE
    1009:   CANNOT BLINK DEBUG LEDS
    1010:   CANNOT CREATE LEDC CHANNELS
    2000:   CANNOT CREATE MAIN LOOP TASK
    2001:   CANNOT CREATE DEBUG LED LOOP TASK
*/

#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <driver/dac_oneshot.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

/*
    Type definitions
*/

typedef struct
{
    float kalmanAngle;
    float kalmanError;
} kalman_struct_1d_t;

typedef struct
{
    float output;
    float error;
    float prevError;
} pid_output_t;

typedef struct
{
    int8_t sbRoll;
    int8_t sbPitch;
    int8_t sbYaw;
    int8_t sbThrottle;
    int8_t sbMode;
} comm_protocol_structure_t;

typedef enum
{
    DEBUG_LED_FRONT,
    DEBUG_LED_BACK,
} debug_led_t;

typedef struct
{
    debug_led_t LED;
    uint32_t count;
    uint32_t delayMS;
} debug_led_command_t;

// DEFINE CONSTANTS HERE THAT WILL BE USED THROUGHOUT THE PROGRAM LOGIC CODE

// debug constants and structures
static const char *BOOT_MSG = "ARCHIE\n\0";
static const size_t BOOT_MSG_LEN = 8;

// UART

static const uart_port_t uart_debug_port = UART_NUM_0;
static const uart_config_t uart_debug_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT};
static const int uart_debug_buffer_size = (1024 * 2);
static QueueHandle_t uart_queue_handle;

// DAC (for checking frequency)

static uint64_t DBG_DAC_CYCLE_TIME;

// Quadcopter system refresh rate
static const uint32_t SYSTEM_REFRESH_RATE = (250);                    // 250Hz
static const uint32_t SRR_CYCLE_WIDTH = (1000 / SYSTEM_REFRESH_RATE); // 1000ms / refresh rate, milliseconds
static const uint32_t SRR_CYCLE_WIDTH_MICRO = (SRR_CYCLE_WIDTH * 1000);
static const float SRR_CYCLE_WIDTH_SECONDS = (SRR_CYCLE_WIDTH / 1000.0f);

//

// I2C master device constants and structures
#define SCL_GPIO (23)
#define SDATA_GPIO (22)
#define MASTER_FREQUENCY (400000)
#define ESP_I2C_PORT (0)

static const i2c_master_bus_config_t mcu_master_device_config = {
    .clk_source = I2C_CLK_SRC_APB,
    .i2c_port = ESP_I2C_PORT,
    .scl_io_num = SCL_GPIO,
    .sda_io_num = SDATA_GPIO,
    .glitch_ignore_cnt = 7, // default
    // .flags.enable_internal_pullup = true
};
static i2c_master_bus_handle_t mcu_master_device_handle;

// MPU6050-related constants and structures
static const float RCF = SRR_CYCLE_WIDTH / 65.5f * 1 / 1000.0f;
static const i2c_device_config_t mpu6050_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,
    .scl_speed_hz = 400000,
    // .scl_wait_us = 0xFFFFF
};
static i2c_master_dev_handle_t mpu6050_handle;

// LED CODE
static const unsigned long LED_FRONT_BASE = (1ULL << 15);
static const unsigned long LED_BACK_BASE = (1ULL << 5);
static const gpio_config_t gpio_led_front_config = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = LED_FRONT_BASE,
    .pull_down_en = 0,
    .pull_up_en = 0,
};
static const gpio_config_t gpio_led_back_config = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = LED_BACK_BASE,
    .pull_down_en = 0,
    .pull_up_en = 0,
};
QueueHandle_t debug_led_queue_handle;

// OFFSETS for sensor readings

static const float fOffsetAccelX = 0.06646f;
static const float fOffsetAccelY = 0.01325f;
static const float fOffsetAccelZ = 0.18885f;

static const float fOffsetRoll = 0.16798f;  // 0.16798f;
static const float fOffsetPitch = 0.82442f; // 0.82442f
static const float fOffsetYaw = 0.58017f;

static const uint8_t mpu6050_pmc_reg[2] = {0x6B, 0x00};               // set to continuous mode
static const uint8_t mpu6050_gyro_scale_factor_reg[2] = {0x1B, 0x08}; // 65.5 Mode
static const uint8_t mpu6050_acce_scale_factor_reg[2] = {0x1C, 0x10};
static const uint8_t mpu6050_gyro_control_reg[1] = {0x43};
static const uint8_t mpu6050_acce_control_reg[1] = {0x3B};
static const uint8_t mpu6050_sensor_low_pass_filter_reg[2] = {0x1A, 0x05};

static float fRoll = 0.0f, fPitch = 0.0f, fYaw = 0.0f;
static int16_t sdRoll, sdPitch, sdYaw;
static uint8_t gyro_results_buffer[6];

static uint8_t accel_results_buffer[6];
static float fAccelX, fAccelY, fAccelZ;
static int16_t sdAccelX, sdAccelY, sdAccelZ;
static float fAccelPitch, fAccelRoll;
static const float RAD_TO_DEG = 180.0f / 3.14159f;
// static const float DEG_TO_RAD = 3.14159f / 180.0f;

// MPU6050 Kalman variables
static float fKalmanAngleRoll = 0.0f;              // Initial guess is 0 because quadcopter will take off at a level surface
static float fKalmanRollAngleUncertainty = 2 * 2;  // Uncertainty of initial angle (since there are no real leveled surface) is 2 degrees
static float fKalmanAnglePitch = 0.0f;             // Initial guess is 0 because quadcopter will take off at a level surface
static float fKalmanPitchAngleUncertainty = 2 * 2; // Uncertainty of initial angle (since there are no real leveled surface) is 2 degrees
static kalman_struct_1d_t xKalmanOutput = {
    0.0f,
    0.0f};

static const float fKalmanRotationRateVariance = 4 * 4;  // Real life scenario error of 4 degrees / seconds; variance of rotation rate at the current iteration.
static const float fKalmanAccelerometerVariance = 3 * 3; // Real life scenario error of 3 degrees; variance of accelerometer at the current iteration.

// UART DEBUGGING constants and structures
static uart_port_t uart_debugging_port = UART_NUM_0; // directs to standard output

// Electronic Speed Controller (ESC) related constants
// static const int32_t ESC_LOW_VALUE = 65536;                 // 2^18 * 0.25; 0.25 is the pulse width ratio with a PWM frequency of 250Hz.
// static const int32_t ESC_HIGH_VALUE = 131072;               // 2^18 * 0.50; 0.50 is the pulse width ratio with a PWM frequency of 250Hz.
// static const int32_t ESC_HIGH_THROTTLE_SAFE_VALUE = 104858; // ESC_HIGH_VALUE * 0.80, only 80% power of motor will be used to prevent over saturation.
// static const int32_t ESC_ANGULAR_SAFE_VALUE = 131000;// Rounded off to 131000, this is as a safe value for angular motions like roll, pitch, and yaw.

ledc_timer_config_t motors_main_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_18_BIT, // 18 bits resolution
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 250, // 250Hz
    .clk_cfg = LEDC_AUTO_CLK, // Use the default clock source
};
ledc_channel_config_t front_left_motor = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_NUM_4, // GPIO 25 is used for debugging
    .duty = 65536, // Initial duty cycle is 0
};
ledc_channel_config_t front_right_motor = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_NUM_18, // GPIO 25 is used for debugging
    .duty = 65536, // Initial duty cycle is 0
};
ledc_channel_config_t rear_left_motor = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_NUM_19, // GPIO 25 is used for debugging
    .duty = 65536, // Initial duty cycle is 0
};
ledc_channel_config_t rear_right_motor = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_NUM_21, // GPIO 25 is used for debugging
    .duty = 65536, // Initial duty cycle is 0
};

// PID related constants
static comm_protocol_structure_t receivedData;
float fDesiredRollAngle, fDesiredPitchAngle, fDesiredYawAngle;
float fErrorRollAngle, fErrorPitchAngle, fErrorYawAngle;
float fPIDOutput;

// FUNCTIONS AND TASKS

/// @brief The kalman calculation algorithm for the angles combining gyro and accelerometer calculations.
/// @param fpKalmanOutput       [out] The array (should be of length 2) that will contain the kalman algorithm calculation. First element = Kalman Angle, Second element = Kalman Angle Uncertainty.
/// @param fKalmanState         [in]  Kalman Angle.
/// @param fKalmanUncertainty   [in]  Kalman Uncertainty.
/// @param fKalmanInput         [in]  Rate of Gyro at the current iteration.
/// @param fKalmanMeasurement   [in]  Accelerometer measurement at the current iteration.
/// @return void
void IRAM_ATTR kalman_1d_mpu6050(kalman_struct_1d_t *pKalmanOutput, float fKalmanState, float fKalmanUncertainty, float fKalmanInput, float fKalmanMeasurement)
{

    fKalmanState = fKalmanState + (SRR_CYCLE_WIDTH_SECONDS * fKalmanInput);

    fKalmanUncertainty = fKalmanUncertainty + (SRR_CYCLE_WIDTH * SRR_CYCLE_WIDTH * fKalmanRotationRateVariance);

    float fKalmanGain = fKalmanUncertainty * 1 / (1 * fKalmanUncertainty + fKalmanAccelerometerVariance);

    fKalmanState = fKalmanState + fKalmanGain * (fKalmanMeasurement - fKalmanState);

    fKalmanUncertainty = (1 - fKalmanGain) * fKalmanUncertainty;

    pKalmanOutput->kalmanAngle = fKalmanState;
    pKalmanOutput->kalmanError = fKalmanUncertainty;
}

/// @brief 
/// @param fError 
/// @param fP 
/// @param fI 
/// @param fD 
/// @param fPrevError 
/// @param fPrevIterm 
/// @return 
void IRAM_ATTR PID_controller_function(float fError, float fP, float fI, float fD, float fPrevError, float fPrevIterm)
{

    float fPTerm = fP * fError;
    float fIterm = fPrevIterm + fI * (fError + fPrevError) * SRR_CYCLE_WIDTH_SECONDS / 2;
    // if (fIterm > ESC_HI)
}

/// @brief Use to output debugging details in the main UART port
/// @param string the string to be printed, null-terminated.
/// @param len the length of the string to be printed, include null byte.
/// @return 
void IRAM_ATTR debug_print(const char *string, size_t len)
{
    if (uart_write_bytes(uart_debugging_port, (const void *)string, len) < 0)
    {
        printf("error outputting values to uart port, probably 0, see source code\n");
    }
}

/// @brief Use to blink the front debugging leds.
/// @param count the amount of blinks to do in the led.
/// @param delayMS the amount of delay in ms before the led turns off.
/// @return void
void IRAM_ATTR debug_led_front_blink(int count, int delayMS)
{
    for (int i = 0; i < count; i++)
    {
        printf("Blinking front LEDS\n");
        gpio_set_level(GPIO_NUM_15, 1);
        vTaskDelay(pdMS_TO_TICKS(delayMS));
        gpio_set_level(GPIO_NUM_15, 0);
        vTaskDelay(pdMS_TO_TICKS(delayMS));
    }
}

/// @brief Use to blink the back debugging leds.
/// @param count the amount of blinks to do in the led.
/// @param delayMS the amount of delay in ms before the led turns off.
/// @return void
void IRAM_ATTR debug_led_back_blink(int count, int delayMS)
{
    for (int i = 0; i < count; i++)
    {
        printf("Blinking back LEDS\n");
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(pdMS_TO_TICKS(delayMS));
        gpio_set_level(GPIO_NUM_5, 0);
        vTaskDelay(pdMS_TO_TICKS(delayMS));
    }
}

/// @brief Main gpio task loop that handles all blinking operations of all debugging leds
/// @param pvParameters 
/// @return void
void IRAM_ATTR led_gpio_task_loop(void *pvParameters)
{
    BaseType_t status;
    debug_led_command_t received_command;
    for (;;)
    {
        status = xQueueReceive(debug_led_queue_handle, &received_command, portMAX_DELAY);
        if (status == pdPASS)
        {
            if (received_command.LED == DEBUG_LED_FRONT)
            {
                debug_led_front_blink(received_command.count, received_command.delayMS);
            }
            else
            {
                debug_led_back_blink(received_command.count, received_command.delayMS);
            }
        }
        else
        {
            char *msg = "1009\n\0";
            debug_print(msg, 6);
        }
    }
}

/// @brief Main flight controller loop task
/// @param pvParameters 
/// @return void
void IRAM_ATTR flight_controller_loop(void *pvParameters)
{
    /*
    this will be used as the main program loop for the flight controller, it will contain all the logic code
    of the flight controller. This always run at 250Hz.
    */
    for (;;)
    {
        // Set frequency of GPIO 25 to 3.3v, use this to debug SRR of quadcopter

        // set 4MS delay to match 250Hz
        DBG_DAC_CYCLE_TIME = esp_timer_get_time() + SRR_CYCLE_WIDTH_MICRO;

        // Receive Accel data
        i2c_master_transmit_receive(mpu6050_handle, mpu6050_acce_control_reg, 1, accel_results_buffer, 6, 1000);
        // Receive gyro data
        i2c_master_transmit_receive(mpu6050_handle, mpu6050_gyro_control_reg, 1, gyro_results_buffer, 6, 1000);

        // Get separate gyro axes data
        sdPitch = gyro_results_buffer[2] << 8 | gyro_results_buffer[3]; // Y
        sdRoll = gyro_results_buffer[0] << 8 | gyro_results_buffer[1];  // X
        sdYaw = gyro_results_buffer[4] << 8 | gyro_results_buffer[5];   // Z

        // Get separate accel axes data
        sdAccelX = accel_results_buffer[0] << 8 | gyro_results_buffer[1];
        sdAccelY = accel_results_buffer[2] << 8 | gyro_results_buffer[3];
        sdAccelZ = accel_results_buffer[4] << 8 | gyro_results_buffer[5];

        // Convert to 1G
        fAccelX = sdAccelX / 4096.0f - fOffsetAccelX;
        fAccelY = sdAccelY / 4096.0f - fOffsetAccelY;
        fAccelZ = sdAccelZ / 4096.0f - fOffsetAccelZ;

        // Calculate Pitch and Roll through accelerometer axes
        fAccelRoll = atan(fAccelY / (sqrt(fAccelX * fAccelX + fAccelZ * fAccelZ))) * RAD_TO_DEG;
        fAccelPitch = -atan(fAccelX / (sqrt(fAccelY * fAccelY + fAccelZ * fAccelZ))) * RAD_TO_DEG;

        // Get angle through gyro and add them (integration);
        // RCF constant contains conversion to velocity (i.e. gyro reading at axis / 65.5 = velocity at axis)

        fRoll += ((sdRoll - fOffsetRoll) * RCF);
        fPitch += ((sdPitch - fOffsetPitch) * RCF);
        fYaw += ((sdYaw - fOffsetYaw) * RCF);

        // printf("Roll\t\t%0.f\nPitch\t\t%0.f\nYaw\t\t%0.f\n", fRoll, fPitch, fYaw);
        // printf("%.2f\n", fPitch);
        // printf("GravityVec%0.4f\n", fAccelGravityVector);
        // printf("AccelX%0.2fAccelY%0.2fAccelZ%0.2f\n", fAccelX, fAccelY, fAccelZ);
        // printf("AccelX\t%0.2f\nAccelY%0.2f\n", fAccelX, fAccelY);
        // printf("Roll%0.fPitch%0.fAccelZ%0.f\n", fAccelRoll, fAccelPitch, fAccelZ);
        // printf("Roll\t\t%0.f\nPitch\t\t%0.f\n", fAccelRoll, fAccelPitch);

        // Kalman Filter combining Accelerometer with Gyroscope

        // Roll
        kalman_1d_mpu6050(&xKalmanOutput, fKalmanAngleRoll, fKalmanRollAngleUncertainty, ((float)sdRoll / 65.5f) - fOffsetRoll, fAccelRoll);
        fKalmanAngleRoll = xKalmanOutput.kalmanAngle;
        fKalmanRollAngleUncertainty = xKalmanOutput.kalmanError;

        // Pitch
        kalman_1d_mpu6050(&xKalmanOutput, fKalmanAnglePitch, fKalmanPitchAngleUncertainty, ((float)sdPitch / 65.5f) - fOffsetPitch, fAccelPitch);
        fKalmanAnglePitch = xKalmanOutput.kalmanAngle;
        fKalmanPitchAngleUncertainty = xKalmanOutput.kalmanError;

        // printf("Roll Rate\t%5.5f\n", ((float)sdRoll / 65.5f) - fOffsetRoll);
        // printf("Pitch Rate\t%5.5f\n", ((float)sdPitch / 65.5f) - fOffsetPitch);

        // printf("ROLL%ld,PITCH%ld\r\n", (int32_t)fKalmanAngleRoll, (int32_t)fKalmanAnglePitch);

        // Set GPIO voltage to 0v, use this to check SRR of quadcopter.
        // Busy loop to achieve system refresh rate
        while (esp_timer_get_time() < DBG_DAC_CYCLE_TIME)
            ;
    }
}

void app_main(void)
{
    /*
    this will be used as the setup function which will setup all required resources and peripherals for the
    flight controller
    */

    // debug peripherals initialization

    // UART
    if (uart_param_config(uart_debug_port, &uart_debug_config) != ESP_OK)
    {
        printf("Cannot initialize debugging UART device\n");
    }

    if (uart_driver_install(uart_debug_port, uart_debug_buffer_size, uart_debug_buffer_size, 10, &uart_queue_handle, 0) != ESP_OK)
    {
        printf("Cannot initialize UART driver for debugging\n");
    }

    //Setup ESC
    if (ledc_timer_config(&motors_main_timer) != ESP_OK)
    {
        printf("Cannot initialize motors main timer\n");
        char *msg = "1005\n\0";
        debug_print(msg, 6);
    }

    if (ledc_channel_config(&front_left_motor) != ESP_OK)
    {
        printf("Cannot initialize front left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_channel_config(&front_right_motor) != ESP_OK)
    {
        printf("Cannot initialize front right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_channel_config(&rear_left_motor) != ESP_OK)
    {
        printf("Cannot initialize rear left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_channel_config(&rear_right_motor) != ESP_OK)
    {
        printf("Cannot initialize rear right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    //Set duty cycle to 13107, which is 5% of the maximum duty cycle of 2^18
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 65535) != ESP_OK)
    {
        printf("Cannot set duty cycle for front left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 65535) != ESP_OK)
    {
        printf("Cannot set duty cycle for front right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 65535) != ESP_OK)
    {
        printf("Cannot set duty cycle for rear left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 65535) != ESP_OK)
    {
        printf("Cannot set duty cycle for rear right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    // Start the LEDC channels
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK)
    {
        printf("Cannot update duty cycle for front left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1) != ESP_OK)
    {
        printf("Cannot update duty cycle for front right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2) != ESP_OK)
    {
        printf("Cannot update duty cycle for rear left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3) != ESP_OK)
    {
        printf("Cannot update duty cycle for rear right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }


    // Sleep for 1 second to allow the motors to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 101908) != ESP_OK)
    {
        printf("Cannot set duty cycle for front left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 101908) != ESP_OK)
    {
        printf("Cannot set duty cycle for front right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 101908) != ESP_OK)
    {
        printf("Cannot set duty cycle for rear left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 101908) != ESP_OK)
    {
        printf("Cannot set duty cycle for rear right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    // Start the LEDC channels
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK)
    {
        printf("Cannot update duty cycle for front left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1) != ESP_OK)
    {
        printf("Cannot update duty cycle for front right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2) != ESP_OK)
    {
        printf("Cannot update duty cycle for rear left motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3) != ESP_OK)
    {
        printf("Cannot update duty cycle for rear right motor\n");
        char *msg = "1010\n\0";
        debug_print(msg, 6);
    }
    

    // print boot message
    debug_print(BOOT_MSG, BOOT_MSG_LEN);

    // setup debug led queue handle
    debug_led_queue_handle = xQueueCreate(5, sizeof(debug_led_command_t));

    if (debug_led_queue_handle == NULL)
    {
        // set debugging LEDS
        char *msg = "1008\n\0";
        debug_print(msg, 6);
    }

    // setup gpio for setting base current for the LEDS
    if (gpio_config(&gpio_led_front_config) != ESP_OK)
    {
        // set debugging LEDS
        char *msg = "1007\n\0";
        debug_print(msg, 6);
    }

    if (gpio_config(&gpio_led_back_config) != ESP_OK)
    {
        // set debugging LEDS
        char *msg = "1007\n\0";
        debug_print(msg, 6);
    }

    // Initialize sensors
    if (i2c_new_master_bus(&mcu_master_device_config, &mcu_master_device_handle) != ESP_OK)
    {
        char *msg = "1000\n\0";
        debug_print(msg, 6);
    }

    if (i2c_master_bus_add_device(mcu_master_device_handle, &mpu6050_config, &mpu6050_handle) != ESP_OK)
    {
        // set debugging LEDS
        char *msg = "1001\n\0";
        debug_print(msg, 6);
    }

    // Initialize mpu6050 sensor internal register configs
    if (i2c_master_transmit(mpu6050_handle, mpu6050_pmc_reg, 2, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        // wait for 100MS
        char *msg = "1002\n\0";
        debug_print(msg, 6);
    }

    if (i2c_master_transmit(mpu6050_handle, mpu6050_gyro_scale_factor_reg, 2, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        // wait for 100MS
        char *msg = "1003\n\0";
        debug_print(msg, 6);
    }

    if (i2c_master_transmit(mpu6050_handle, mpu6050_acce_scale_factor_reg, 2, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        // wait for 100MS
        char *msg = "1004\n\0";
        debug_print(msg, 6);
    }

    if (i2c_master_transmit(mpu6050_handle, mpu6050_sensor_low_pass_filter_reg, 2, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        // wait for 100MS
        char *msg = "1006\n\0";
        debug_print(msg, 6);
    }

    if (xTaskCreatePinnedToCore(flight_controller_loop, "FC_TASK", 10000, NULL, 1, NULL, 1) != pdPASS)
    {
        char *msg = "2000\n\0";
        debug_print(msg, 6);
    }

    if (xTaskCreatePinnedToCore(led_gpio_task_loop, "DBG_LED_TASK<", 10000, NULL, 1, NULL, 0) != pdPASS)
    {
        char *msg = "2001\n\0";
        debug_print(msg, 6);
    }

    debug_led_command_t front_led_command = {
        .LED = DEBUG_LED_FRONT,
        .count = 5,
        .delayMS = 500,
    };
    if (xQueueSend(debug_led_queue_handle, &front_led_command, 0) != pdTRUE)
    {
        char *msg = "1008\n\0";
        debug_print(msg, 6);
    }

    debug_led_command_t back_led_command = {
        .LED = DEBUG_LED_BACK,
        .count = 5,
        .delayMS = 500,
    };
    if (xQueueSend(debug_led_queue_handle, &back_led_command, 0) != pdTRUE)
    {
        char *msg = "1008\n\0";
        debug_print(msg, 6);
    }
}