#ifndef BMI088_H
#define BMI088_H

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "state.h"
#include "filter.h"

esp_err_t bmi088_accel_read(uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t bmi088_register_write(spi_device_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

void spi_init();

void bmi088_setup();

void bmi088_calibrate(KF *kf);

void bmi088_update(State *state, float dt);

#endif
