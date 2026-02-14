#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"

/* ===================== CONFIG ===================== */

#define I2C_MASTER_SCL_IO           GPIO_NUM_38
#define I2C_MASTER_SDA_IO           GPIO_NUM_37
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_WHO_AM_I_REG_ADDR   0x75
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B
#define MPU6050_ACCEL_REG_ADDR      0x3B
#define MPU6050_GYRO_REG_ADDR       0x43

#define ALPHA 0.98f                 // Complementary filter factor

/* ===================== STATE ===================== */

float ACCEL_OFFSET_X = 0.0f;
float ACCEL_OFFSET_Y = 0.0f;
float ACCEL_OFFSET_Z = 0.0f;

float GYRO_OFFSET_X  = 0.0f;
float GYRO_OFFSET_Y  = 0.0f;

/* ===================== I2C HELPERS ===================== */

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev,
                                uint8_t reg,
                                uint8_t *data,
                                size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len,
                                       I2C_MASTER_TIMEOUT_MS);
}

esp_err_t mpu6050_register_write(i2c_master_dev_handle_t dev,
                                 uint8_t reg,
                                 uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev, buf, sizeof(buf),
                               I2C_MASTER_TIMEOUT_MS);
}

/* ===================== I2C INIT ===================== */

void i2c_master_init(i2c_master_bus_handle_t *bus,
                     i2c_master_dev_handle_t *dev)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, bus));

    i2c_device_config_t dev_cfg = {
        .device_address = MPU6050_SENSOR_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &dev_cfg, dev));
}

/* ===================== MPU SETUP ===================== */

void mpu6050_setup(i2c_master_dev_handle_t dev, uint8_t *data)
{
    ESP_ERROR_CHECK(mpu6050_register_write(dev, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00));

    ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI("MPU6050", "WHO_AM_I = 0x%X", data[0]);

    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x19, 0x07)); // 125 Hz
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1A, 0x03)); // DLPF ~42 Hz
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1C, 0x00)); // ±2g
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1B, 0x00)); // ±250°/s
}

/* ===================== CALIBRATION ===================== */

void mpu6050_calibrate(i2c_master_dev_handle_t dev, uint8_t *data)
{
    ESP_LOGI("MPU6050", "Calibrating... keep sensor still");

    for (int i = 0; i < 2000; i++) {
        ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_ACCEL_REG_ADDR, data, 6));
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];

        ACCEL_OFFSET_X += ax;
        ACCEL_OFFSET_Y += ay;
        ACCEL_OFFSET_Z += (az - 16384);

        ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_GYRO_REG_ADDR, data, 6));
        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];

        GYRO_OFFSET_X += gx;
        GYRO_OFFSET_Y += gy;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ACCEL_OFFSET_X /= 2000.0f;
    ACCEL_OFFSET_Y /= 2000.0f;
    ACCEL_OFFSET_Z /= 2000.0f;
    GYRO_OFFSET_X  /= 2000.0f;
    GYRO_OFFSET_Y  /= 2000.0f;

    ESP_LOGI("MPU6050", "Calibration done");
}

/* ===================== SENSOR UPDATE ===================== */

void mpu6050_update(i2c_master_dev_handle_t dev, i2c_master_bus_handle_t bus, uint8_t *data, State *state, float dt)
{
    while (mpu6050_register_read(dev, MPU6050_ACCEL_REG_ADDR, data, 6) != ESP_OK) {
        i2c_master_bus_reset(bus);
    }

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    float xg = ((float)ax - ACCEL_OFFSET_X) / 16384.0f;
    float yg = ((float)ay - ACCEL_OFFSET_Y) / 16384.0f;
    float zg = ((float)az - ACCEL_OFFSET_Z) / 16384.0f;

    float accel_roll  = atan2f(yg, sqrtf(xg * xg + zg * zg)) * 180.0f / M_PI;
    float accel_pitch = atan2f(-xg, sqrtf(yg * yg + zg * zg)) * 180.0f / M_PI;

    while (mpu6050_register_read(dev, MPU6050_GYRO_REG_ADDR, data, 6) != ESP_OK) {
        i2c_master_bus_reset(bus);
    }

    int16_t gx = (data[0] << 8) | data[1];
    int16_t gy = (data[2] << 8) | data[3];

    float gyro_x = ((float)gx - GYRO_OFFSET_X) / 131.0f;
    float gyro_y = ((float)gy - GYRO_OFFSET_Y) / 131.0f;

    state->m_angle[0] = ALPHA * (state->m_angle[0] + gyro_x * dt) + (1.0f - ALPHA) * accel_roll;
    state->m_angle[1] = ALPHA * (state->m_angle[1] + gyro_y * dt) + (1.0f - ALPHA) * accel_pitch;

    state->angular_velocity[0] = gyro_x;
    state->angular_velocity[1] = gyro_y;
}
