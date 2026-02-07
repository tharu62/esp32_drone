#ifndef INPUT_RECEIVER_H
#define INPUT_RECEIVER_H

#include <string.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

void init_espnow();

void get_control_inputs(float *out_throttle, float *out_roll, float *out_pitch);

#endif // INPUT_RECEIVER_H