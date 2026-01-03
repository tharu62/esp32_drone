#include "input_receiver.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define DRONE_TAG "ESP_NOW_DRONE"

float throttle = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

typedef struct __attribute__((packed)) 
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} control_packet_t;

// Callback for received ESP-NOW packets
void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    if (data_len != sizeof(control_packet_t)) return;

    control_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    throttle = pkt.throttle;
    roll     = pkt.roll;
    pitch    = pkt.pitch;
    yaw      = pkt.yaw;

    ESP_LOGI(DRONE_TAG, "CTRL T=%.2f R=%.2f P=%.2f Y=%.2f", throttle, roll, pitch, yaw);
}

void init_espnow() 
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
}

// Accessor function
void get_control_inputs(float *out_throttle, float *out_roll, float *out_pitch, float *out_yaw) 
{
    *out_throttle = throttle;
    *out_roll = roll;
    *out_pitch = pitch;
    *out_yaw = yaw;
}
