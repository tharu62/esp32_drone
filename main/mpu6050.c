#include "mpu6050.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_18                 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_19                 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR         0x68                        /*!< Address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR   0x75                        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B                        /*!< Register addresses of the power management register */
#define MPU6050_RESET_BIT           7
#define MPU6050_ACCELEROMETER_DATA_REG_ADDR 0x3B               /*!< Register address where accelerometer data starts */
#define MPU6050_GYROSCOPE_DATA_REG_ADDR     0x43               /*!< Register address where gyroscope data starts */

float ROLL_GYRO_CALIBRATION_OFFSET   = 0.0f;
float PITCH_GYRO_CALIBRATION_OFFSET  = 0.0f;
float YAW_GYRO_CALIBRATION_OFFSET    = 0.0f;

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void mpu6050_calibration(i2c_master_dev_handle_t dev_handle, uint8_t *data)
{
    for(int i=0; i<2000; i++){
        // ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_ACCELEROMETER_DATA_REG_ADDR, data, 6));

        ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_GYROSCOPE_DATA_REG_ADDR, data, 6));
        int16_t GYRO_RAWX = (data[0] << 8) | data[1];
        int16_t GYRO_RAWY = (data[2] << 8) | data[3];
        int16_t GYRO_RAWZ = (data[4] << 8) | data[5];
        ROLL_GYRO_CALIBRATION_OFFSET     += (float) GYRO_RAWX / 131.0f;
        PITCH_GYRO_CALIBRATION_OFFSET    += (float) GYRO_RAWY / 131.0f;
        YAW_GYRO_CALIBRATION_OFFSET      += (float) GYRO_RAWZ / 131.0f;

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    ROLL_GYRO_CALIBRATION_OFFSET     /= 2000.0f;
    PITCH_GYRO_CALIBRATION_OFFSET    /= 2000.0f;  
    YAW_GYRO_CALIBRATION_OFFSET      /= 2000.0f;
}

void mpu6050_setup(i2c_master_dev_handle_t dev_handle, uint8_t *data)
{
    // Wake up the MPU6050 by writing 0 to the power management register
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00));

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI("mpu6050", "WHO_AM_I = %X", data[0]);
    
    // Set sample rate to 1kHz/(1+7) = 125Hz
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, 0x19, 0x07)); 

    // Set accelerometer configuration to +/- 2g
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, 0x1C, 0x00));
    // Set gyroscope configuration to +/- 250 degrees/second 
    ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, 0x1B, 0x00));
}

void mpu6050_get_angle(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, uint8_t *data, State *state)
{
    ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_ACCELEROMETER_DATA_REG_ADDR, data, 6));
    while (mpu6050_register_read(dev_handle, MPU6050_ACCELEROMETER_DATA_REG_ADDR, data, 6) != ESP_OK) {
        ESP_LOGI("mpu6050", "Failed to read accelerometer data, retrying...");
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(1 / portTICK_PERIOD_MS); // Wait and retry after 1 millisecond if read fails
    }
    // if (mpu6050_register_read(dev_handle, MPU6050_ACCELEROMETER_DATA_REG_ADDR, data, 6) != ESP_OK) {
    //     return;
    // }

    int16_t ACCEL_RAW[3] = {0};
    ACCEL_RAW[0] = (data[0] << 8) | data[1];
    ACCEL_RAW[1] = (data[2] << 8) | data[3];
    ACCEL_RAW[2] = (data[4] << 8) | data[5];
    
    // UPDATED FOR ±2g RANGE  → 16384 LSB/g
    float xg = (float) ACCEL_RAW[0] / 16384.0f;
    float yg = (float) ACCEL_RAW[1] / 16384.0f;
    float zg = (float) ACCEL_RAW[2] / 16384.0f;

    // if (sqrtf(xg * xg + yg * yg + zg * zg) < 0.5f || sqrtf(xg * xg + yg * yg + zg * zg) > 2.0f) {
    //     // Invalid accelerometer reading, likely due to free fall or extreme motion
    //     return;
    // }
    // Reading higher than 1.2g indicates external acceleration, apply simple correction based on percentage and low-pass filter
    if (sqrtf(xg * xg + yg * yg + zg * zg) > 1.0f) {
        float modulus = sqrtf(xg * xg + yg * yg + zg * zg);
        float daccel = modulus - 1.0f;
        xg = 0.97f * (xg - xg * daccel * 100.f / modulus) + 0.03f * xg;
        yg = 0.97f * (yg - yg * daccel * 100.f / modulus) + 0.03f * yg;
    }

    state->m_angle[0] = atan2f(yg, sqrtf(xg * xg + zg * zg)) * 180.0f / M_PI;
    state->m_angle[1] = atan2f(-xg, sqrtf(yg * yg + zg * zg)) * 180.0f / M_PI;
}

void mpu6050_get_rotation_rate(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, uint8_t *data, State *state, float dt)
{
    // ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_GYROSCOPE_DATA_REG_ADDR, data, 6));
    while (mpu6050_register_read(dev_handle, MPU6050_GYROSCOPE_DATA_REG_ADDR, data, 6) != ESP_OK) {
        ESP_LOGI("mpu6050", "Failed to read gyroscope data, retrying...");
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(1 / portTICK_PERIOD_MS); // Wait and retry after 1 millisecond if read fails
    }
    // if (mpu6050_register_read(dev_handle, MPU6050_GYROSCOPE_DATA_REG_ADDR, data, 6) != ESP_OK) {
    //     return;
    // }

    int16_t GYRO_RAW[3] = {0};
    GYRO_RAW[0] = (data[0] << 8) | data[1];
    GYRO_RAW[1] = (data[2] << 8) | data[3];
    GYRO_RAW[2] = (data[4] << 8) | data[5];

    state->angular_acceleration[0] = (float) ((GYRO_RAW[0] / 131.f - ROLL_GYRO_CALIBRATION_OFFSET)  - state->angular_velocity[0]) / dt; 
    state->angular_acceleration[1] = (float) ((GYRO_RAW[1] / 131.f - PITCH_GYRO_CALIBRATION_OFFSET) - state->angular_velocity[1]) / dt; 
    state->angular_acceleration[2] = (float) ((GYRO_RAW[2] / 131.f - YAW_GYRO_CALIBRATION_OFFSET)   - state->angular_velocity[2]) / dt; 

    state->angular_velocity[0] = (float) GYRO_RAW[0] / 131.f - ROLL_GYRO_CALIBRATION_OFFSET;
    state->angular_velocity[1] = (float) GYRO_RAW[1] / 131.f - PITCH_GYRO_CALIBRATION_OFFSET;
    state->angular_velocity[2] = (float) GYRO_RAW[2] / 131.f - YAW_GYRO_CALIBRATION_OFFSET;
}
