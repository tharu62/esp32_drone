#include "bmi088.h"

static const char *TAG = "bmi088";

/* -------------------------------------------------------------------------
 * Pin mapping (ESP32-S3, exact wiring as provided):
 *   CSB_ACCEL = GPIO7
 *   CSB_GYRO  = GPIO8
 *   MOSI      = GPIO11
 *   MISO      = GPIO13
 *   SCK       = GPIO12
 *   PS        = GND  -> SPI mode selected for the gyroscope part
 *   INT1..4   = not connected (polling mode, no interrupts used)
 *
 * KF struct / calibration is intentionally left unimplemented for this
 * first hardware bring-up pass, per request.
 * ---------------------------------------------------------------------- */

#define PIN_NUM_MOSI     GPIO_NUM_11
#define PIN_NUM_MISO     GPIO_NUM_13
#define PIN_NUM_SCLK     GPIO_NUM_12
#define PIN_NUM_CS_ACCEL GPIO_NUM_7
#define PIN_NUM_CS_GYRO  GPIO_NUM_8

#define BMI088_ACC_CHIP_ID   0x1E
#define BMI088_GYRO_CHIP_ID  0x0F

/* Accelerometer registers */
#define ACC_CHIP_ID     0x00
#define ACC_X_LSB       0x12
#define ACC_CONF        0x40
#define ACC_RANGE       0x41
#define ACC_PWR_CONF    0x7C
#define ACC_PWR_CTRL    0x7D
#define ACC_SOFTRESET   0x7E

/* Gyroscope registers */
#define GYRO_CHIP_ID    0x00
#define RATE_X_LSB      0x02
#define GYRO_RANGE      0x0F
#define GYRO_BANDWIDTH  0x10
#define GYRO_LPM1       0x11
#define GYRO_SOFTRESET  0x14

#define SOFTRESET_CMD   0xB6

/* Accelerometer range: 0x01 = +-6g (reset default), sensitivity 5460 LSB/g */
#define ACC_RANGE_SETTING   0x01

/* Gyroscope range: 0x00 = +-2000 dps (reset default), 16.384 LSB/(deg/s) */
#define GYRO_RANGE_SETTING      0x00
#define GYRO_SENSITIVITY_LSB    16.384f /* LSB per deg/s */

#define G_TO_MS2   9.80665f
#define DEG_TO_RAD (float)(M_PI / 180.0)

/* Gyroscope device handle -- populated by spi_init(), used internally.
 * bmi088.h only exposes one spi_device_handle_t (the accelerometer), so the
 * gyroscope's separate chip-select device is kept file-scope here. */
static spi_device_handle_t gyro_handle = NULL;

/* Accelerometer device handle -- populated by spi_init(), used internally. */
static spi_device_handle_t accel_handle = NULL;

esp_err_t bmi088_accel_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_addr = reg_addr | 0x80; /* bit #0 = 1 -> read */

    /* The accelerometer part inserts one dummy byte between the address
     * byte and the actual data (datasheet 6.1.2). */
    const size_t header_bytes = 2;
    size_t total = header_bytes + len;

    uint8_t tx[total];
    uint8_t rx[total];
    memset(tx, 0x00, total);
    memset(rx, 0x00, total);
    tx[0] = read_addr;

    spi_transaction_t t = {
        .length = 8 * total,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(accel_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI accel read of reg 0x%02X failed: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }

    memcpy(data, &rx[header_bytes], len);
    return ESP_OK;
}

esp_err_t bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_addr = reg_addr | 0x80; /* bit #0 = 1 -> read */

    /* The gyroscope part does NOT use a dummy byte (datasheet 6.1.1). */
    const size_t header_bytes = 1;
    size_t total = header_bytes + len;

    uint8_t tx[header_bytes + len];
    uint8_t rx[header_bytes + len];
    memset(tx, 0x00, total);
    memset(rx, 0x00, total);
    tx[0] = read_addr;

    spi_transaction_t t = {
        .length = 8 * total,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(gyro_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI gyro read of reg 0x%02X failed: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }

    memcpy(data, &rx[header_bytes], len);
    return ESP_OK;
}

esp_err_t bmi088_register_write(spi_device_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_addr = reg_addr & 0x7F; /* bit #0 = 0 -> write */
    uint8_t tx[2] = { write_addr, data };

    spi_transaction_t t = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(dev_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write of reg 0x%02X failed: %s", reg_addr, esp_err_to_name(ret));
    }

    /* Minimum idle time between write accesses in normal mode (datasheet
     * table 12: t_IDLE_wacc = 2us min). Give it a comfortable margin. */
    esp_rom_delay_us(5);

    return ret;
}

void spi_init()
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    /* BMI088 SPI is compatible with mode 0 or mode 3; max clock 10 MHz
     * (datasheet table 12). We use mode 0. */
    spi_device_interface_config_t accel_config = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS_ACCEL,
        .queue_size = 1,
    };

    spi_device_interface_config_t gyro_config = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS_GYRO,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &accel_config, &accel_handle));
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &gyro_config, &gyro_handle));

    ESP_LOGI(TAG, "SPI bus initialized");
}

void bmi088_setup()
{
    uint8_t chip_id = 0;

    /* --- Accelerometer part --- */

    /* The accelerometer always starts in I2C mode after POR, regardless of
     * the PS pin (PS only selects the gyroscope's interface). It must be
     * switched to SPI mode with a dummy SPI read (datasheet chapter 3).
     * The result of this first read is invalid and discarded. */
    bmi088_accel_read(ACC_CHIP_ID, &chip_id, 1);

    /* Soft-reset the accelerometer, then wait for the boot time. */
    bmi088_register_write(accel_handle, ACC_SOFTRESET, SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(1));

    if (bmi088_accel_read(ACC_CHIP_ID, &chip_id, 1) != ESP_OK || chip_id != BMI088_ACC_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected accel chip id: 0x%02X (expected 0x%02X)", chip_id, BMI088_ACC_CHIP_ID);
    } else {
        ESP_LOGI(TAG, "BMI088 accelerometer detected (chip id 0x%02X)", chip_id);
    }

    /* Quick start guide, chapter 3: power up, wait 1ms, enter normal mode,
     * wait 450us. Also make sure we are not in advanced power-save mode
     * (ACC_PWR_CONF = 0x00 = active). */
    bmi088_register_write(accel_handle, ACC_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    bmi088_register_write(accel_handle, ACC_PWR_CTRL, 0x04);
    esp_rom_delay_us(450);

    /* ODR = 100Hz, normal filter (reset default 0xA8), explicit for clarity. */
    bmi088_register_write(accel_handle, ACC_CONF, 0xA8);

    /* Range = +-6g (reset default), explicit for clarity. */
    bmi088_register_write(accel_handle, ACC_RANGE, ACC_RANGE_SETTING);

    /* --- Gyroscope part --- */

    bmi088_register_write(gyro_handle, GYRO_SOFTRESET, SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(30)); /* datasheet: up to 30ms after reset */

    if (bmi088_gyro_read(GYRO_CHIP_ID, &chip_id, 1) != ESP_OK || chip_id != BMI088_GYRO_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected gyro chip id: 0x%02X (expected 0x%02X)", chip_id, BMI088_GYRO_CHIP_ID);
    } else {
        ESP_LOGI(TAG, "BMI088 gyroscope detected (chip id 0x%02X)", chip_id);
    }

    /* Make sure gyro is in normal (fully operational) power mode. */
    bmi088_register_write(gyro_handle, GYRO_LPM1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* Range = +-2000 dps (reset default), explicit for clarity. */
    bmi088_register_write(gyro_handle, GYRO_RANGE, GYRO_RANGE_SETTING);

    /* ODR = 1000Hz, filter BW = 116Hz (gyro_bw = 0x02). */
    bmi088_register_write(gyro_handle, GYRO_BANDWIDTH, 0x02);

    ESP_LOGI(TAG, "BMI088 setup complete");
}

void bmi088_calibrate(KF *kf)
{
    /* Not implemented for this initial hardware bring-up pass. */
    (void)kf;
}

void bmi088_update(State *state, float dt)
{
    (void)dt;

    uint8_t acc_raw[6];
    uint8_t gyro_raw[6];

    esp_err_t ret_a = bmi088_accel_read(ACC_X_LSB, acc_raw, sizeof(acc_raw));
    esp_err_t ret_g = bmi088_gyro_read(RATE_X_LSB, gyro_raw, sizeof(gyro_raw));

    if (ret_a != ESP_OK || ret_g != ESP_OK) {
        ESP_LOGW(TAG, "BMI088 read failed (acc=%s, gyro=%s)", esp_err_to_name(ret_a), esp_err_to_name(ret_g));
        return;
    }

    for (int i = 0; i < 3; i++) {
        int16_t acc_lsb = (int16_t)((acc_raw[i * 2 + 1] << 8) | acc_raw[i * 2]);
        int16_t gyro_lsb = (int16_t)((gyro_raw[i * 2 + 1] << 8) | gyro_raw[i * 2]);

        float acc_mg = ((float)acc_lsb / 32768.0f) * 1000.0f * (float)(1 << (ACC_RANGE_SETTING + 1)) * 1.5f;
        float acc_ms2 = (acc_mg / 1000.0f) * G_TO_MS2;

        float gyro_dps = (float)gyro_lsb / GYRO_SENSITIVITY_LSB;
        float gyro_rads = gyro_dps * DEG_TO_RAD;

        state->a[i] = acc_ms2;
        state->w[i] = gyro_rads;
    }
}