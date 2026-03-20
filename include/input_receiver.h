#ifndef INPUT_RECEIVER_H
#define INPUT_RECEIVER_H

#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

/**
 * @brief Callback function to handle received ESP-NOW packets containing 
 *        control inputs (throttle, roll, pitch).
 */
void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

/**
 * @brief Initializes ESP-NOW for receiving control inputs.
 *        Sets up Wi-Fi in station mode with a fixed MAC address and registers the receive callback.
 */
void init_espnow();

/**
 * @brief Retrieves the latest control inputs received via ESP-NOW.
 */
void get_control_inputs(float *out_throttle, float *out_roll, float *out_pitch);

#endif // INPUT_RECEIVER_H