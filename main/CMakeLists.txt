idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES i2cdev
                       INCLUDE_DIRS "."
                       )
