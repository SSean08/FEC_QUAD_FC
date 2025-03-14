/*



    ERROR CODE REFERENCE:
    1000:   CANNOT INITIALIZE I2C MASTER BUS
    1001:   CANNOT INITIALIZE I2C MPU6050 DEVICE
    1002:   CANNOT SET MPU6050 CONTINUOUS MODE THROUGH INTERNAL REGISTERS
    1003:   CANNOT SET MPU6050 GYROSCOPE SCALE FACTOR THROUGH INTERNAL REGISTERS
    1004:   CANNOT SET MPU6050 ACCELEROMETER SCALE FACTOR THROUGH INTERNAL REGISTERS
    1005:   CANNOT INITIALIZE DAC FREQUENCY CHECKER MODULE
    2000:   CANNOT CREATE MAIN LOOP TASK
*/

#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <driver/i2c_master.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <driver/dac_oneshot.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <math.h>
#include "mpu6050.h"

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

static dac_oneshot_handle_t debug_dac_handle;
static const dac_oneshot_config_t debug_dac_config = {
    .chan_id = DAC_CHAN_0 // GPIO 25
};
static uint64_t DBG_DAC_CYCLE_TIME;

// Quadcopter system refresh rate
#define SYSTEM_REFRESH_RATE (250)                    // 250Hz
#define SRR_CYCLE_WIDTH (1000 / SYSTEM_REFRESH_RATE) // 1000ms / refresh rate, milliseconds
#define SRR_CYCLE_WIDTH_MICRO (SRR_CYCLE_WIDTH * 1000)

//

// I2C master device constants and structures
#define SCL_GPIO (22)
#define SDATA_GPIO (21)
#define MASTER_FREQUENCY (400000)
#define ESP_I2C_PORT (0)

static const i2c_config_t i2c_master_bus_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDATA_GPIO, // select SDA GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_io_num = SCL_GPIO, // select SCL GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = MASTER_FREQUENCY, // select frequency specific to your project
    .clk_flags = 0,                         // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
};
static const int i2c_master_bus_port = 0;

// MPU6050-related constants and structures
// static const float RCF = 4 / 65.5f * 1 / 1000.0f;

// ESPRESSIF MPU6050 driver
static mpu6050_handle_t mpu6050_espressif_handle = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;

// OFFSETS for sensor readings

// static const float fOffsetAccelX = 0.06646f;
// static const float fOffsetAccelY = 0.01325f;
// static const float fOffsetAccelZ = 0.18885f;

// static const float fOffsetRoll =     0.16798f; //0.16798f;
// static const float fOffsetPitch =    0.82442f; //0.82442f
// static const float fOffsetYaw =      0.58017f;

// static const uint8_t mpu6050_pmc_reg[2] = {0x6B, 0x00};               // set to continuous mode
// static const uint8_t mpu6050_gyro_scale_factor_reg[2] = {0x1B, 0x08}; // 65.5 Mode
// static const uint8_t mpu6050_acce_scale_factor_reg[2] = {0x1C, 0x10};
// static const uint8_t mpu6050_gyro_control_reg[1] = {0x43};
// static const uint8_t mpu6050_acce_control_reg[1] = {0x3B};

// static float fRoll = 0.0f, fPitch = 0.0f, fYaw = 0.0f;
// static int16_t sdRoll, sdPitch, sdYaw;
// static uint8_t gyro_results_buffer[6];

// static uint8_t accel_results_buffer[6];
// static float fAccelX, fAccelY, fAccelZ;
// static int16_t sdAccelX, sdAccelY, sdAccelZ;
// static float fAccelGravityVector;
// static float fAccelPitch, fAccelRoll;
// static const float RAD_TO_PI = 180.0f / 3.14159f;

// UART DEBUGGING constants and structures
static uart_port_t uart_debugging_port = UART_NUM_0; // directs to standard output

// FUNCTIONS AND TASKS

static void IRAM_ATTR mpu6050_init()
{
    mpu6050_espressif_handle = mpu6050_create(0, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_espressif_handle, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_espressif_handle);
}

static void IRAM_ATTR mpu6050_read(void *pvParameters)
{
    mpu6050_get_acce(mpu6050_espressif_handle, &acce);
    mpu6050_get_gyro(mpu6050_espressif_handle, &gyro);
    mpu6050_complimentory_filter(mpu6050_espressif_handle, &acce, &gyro, &complimentary_angle);
}

void IRAM_ATTR debug_print(const char *string, size_t len)
{
    if (uart_write_bytes(uart_debugging_port, (const void *)string, len) < 0)
    {
        printf("error outputting values to uart port, probably 0, see source code\n");
    }
}

void IRAM_ATTR flight_controller_loop(void *pvParameters)
{
    /*
    this will be used as the main program loop for the flight controller, it will contain all the logic code
    of the flight controller. This always run at 250Hz.
    */
    for (;;)
    {
        dac_oneshot_output_voltage(debug_dac_handle, 255);

        // set 4MS delay to match 250Hz
        DBG_DAC_CYCLE_TIME = esp_timer_get_time() + SRR_CYCLE_WIDTH_MICRO;
        mpu6050_read(NULL);
        printf("Roll=\t%3.2f\n", complimentary_angle.roll);
        printf("Pitch=\t%3.2f\n", complimentary_angle.pitch);

        dac_oneshot_output_voltage(debug_dac_handle, 0); // should always be last
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

    // DAC FREQUENCY CHECKER
    if (dac_oneshot_new_channel(&debug_dac_config, &debug_dac_handle) != ESP_OK)
    {
        char *msg = "1005\n\0";
        debug_print(msg, 6);
    }

    // print boot message
    debug_print(BOOT_MSG, BOOT_MSG_LEN);

    // Initialize sensors

    // I2C master bus initialization
    if (i2c_param_config(i2c_master_bus_port, &i2c_master_bus_config) != ESP_OK)
    {
        // set debugging LEDS
        char *msg = "1000\n\0";
        debug_print(msg, 6);
    }

    if (i2c_driver_install(i2c_master_bus_port, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)
    {
        // set debugging LEDS
        char *msg = "1000\n\0";
        debug_print(msg, 6);
    }

    // initialize espressif mpu6050 driver
    mpu6050_init();

    if (xTaskCreatePinnedToCore(flight_controller_loop, "FLIGHT_CONTROLLER", 10000, NULL, 1, NULL, 1) != pdPASS)
    {
        char *msg = "2000\n\0";
        debug_print(msg, 6);
    }
}