#include "mpu6050.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_11
#define I2C_MASTER_SDA_IO           GPIO_NUM_10
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000 // 400 kHz
#define I2C_MASTER_TIMEOUT_MS       1000 

#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_WHO_AM_I_REG_ADDR   0x75
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B
#define MPU6050_ACCEL_REG_ADDR      0x3B
#define MPU6050_GYRO_REG_ADDR       0x43
#define ACCELEROMETER_SENSITIVITY   16384.0f // LSB/g for ±2g
#define GYROSCOPE_SENSITIVITY       131.0f   // LSB/(°/second) for ±250°/s

static float ax_offset = 0.0f;
static float ay_offset = 0.0f;
static float az_offset = 0.0f;

static float gx_offset = 0.0f;
static float gy_offset = 0.0f;


esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t mpu6050_register_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}


void i2c_master_init(i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *dev)
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

void mpu6050_setup(i2c_master_dev_handle_t dev)
{
    uint8_t data[10] = {0};
    // ESP_ERROR_CHECK(mpu6050_register_write(dev, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00));
    while(mpu6050_register_write(dev, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00) != ESP_OK){
        ESP_LOGI("MPU6050", "Failed to wake up MPU6050, retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI("MPU6050", "WHO_AM_I = 0x%X", data[0]);

    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x19, 0x07)); // 125 Hz
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1A, 0x03)); // DLPF ~42 Hz
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1C, 0x00)); // ±2g
    ESP_ERROR_CHECK(mpu6050_register_write(dev, 0x1B, 0x00)); // ±250°/s
}


void mpu6050_calibrate(i2c_master_dev_handle_t dev, KF* ekf)
{
    uint8_t data[10] = {0};
    ESP_LOGI("MPU6050", "Calibrating... keep sensor still");

    for (int i = 0; i < 1000; i++) {
        ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_ACCEL_REG_ADDR, data, 6));
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];

        ax_offset += (float)ax; 
        ay_offset += (float)ay; 
        az_offset += (float)az;
        
        ESP_ERROR_CHECK(mpu6050_register_read(dev, MPU6050_GYRO_REG_ADDR, data, 6));
        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];
        // int16_t gz = (data[4] << 8) | data[5]; // unused

        gx_offset += (float)gx;
        gy_offset += (float)gy;

        // 2 ms delay between samples for calibration, total calibration time ~1 seconds
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    ax_offset = ax_offset / 1000.0f;
    ay_offset = ay_offset / 1000.0f;
    az_offset = az_offset / 1000.0f - ACCELEROMETER_SENSITIVITY; // Subtract 1g from Z-axis offset
    ekf->bias[0] = gx_offset / 1000.0f / GYROSCOPE_SENSITIVITY; 
    ekf->bias[1] = gy_offset / 1000.0f / GYROSCOPE_SENSITIVITY; 
    ESP_LOGI("MPU6050", "Calibration done");
}


void mpu6050_update(i2c_master_dev_handle_t dev, i2c_master_bus_handle_t bus, State *state, float dt)
{

    uint8_t data[10] = {0};
    while (mpu6050_register_read(dev, MPU6050_ACCEL_REG_ADDR, data, 6) != ESP_OK) {
        i2c_master_bus_reset(bus);
    }

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    float xg = ((float)ax - ax_offset) / ACCELEROMETER_SENSITIVITY; 
    float yg = ((float)ay - ay_offset) / ACCELEROMETER_SENSITIVITY;
    float zg = ((float)az - az_offset) / ACCELEROMETER_SENSITIVITY;

    // Apply low-pass filter
    // state->a[0] = ALPHA * state->a[0] + (1 - ALPHA) * xg;
    // state->a[1] = ALPHA * state->a[1] + (1 - ALPHA) * yg;
    // state->a[2] = ALPHA * state->a[2] + (1 - ALPHA) * zg;
    
    state->a[0] = xg;
    state->a[1] = yg;
    state->a[2] = zg;

    state->m_angle[0] = atan2f(yg, sqrtf(xg * xg + zg * zg)) * 180.0f / M_PI;
    state->m_angle[1] = atan2f(-xg, sqrtf(yg * yg + zg * zg)) * 180.0f / M_PI;

    while (mpu6050_register_read(dev, MPU6050_GYRO_REG_ADDR, data, 6) != ESP_OK) {
        i2c_master_bus_reset(bus);
    }

    int16_t gx = (data[0] << 8) | data[1];
    int16_t gy = (data[2] << 8) | data[3];
    // int16_t gz = (data[4] << 8) | data[5]; // unused

    state->w[0] = (float)gx / GYROSCOPE_SENSITIVITY;
    state->w[1] = (float)gy / GYROSCOPE_SENSITIVITY;
}
