#include "bmi088.h"


esp_err_t bmi088_register_read(spi_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{

}


esp_err_t bmi088_register_write(spi_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{

}


void spi_init(spi_bus_handle_t *bus_handle, spi_dev_handle_t *dev_handle)
{

}

void bmi088_calibrate(spi_dev_handle_t dev_handle, KF* ekf)
{

}

void bmi088_setup(spi_dev_handle_t dev_handle)
{

}

void bmi088_update(spi_dev_handle_t dev_handle, spi_bus_handle_t bus_handle, State *state, float dt)
{

}