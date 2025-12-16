idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES mpu6050
                       REQUIRES i2cdev
                       INCLUDE_DIRS "."
                       )
