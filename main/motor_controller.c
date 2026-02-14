#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_COUNT 4

#define PWM_FREQ_HZ      6000
#define PWM_RESOLUTION   LEDC_TIMER_12_BIT
#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_TIMER        LEDC_TIMER_0

#define PWM_GPIO_M1 2
#define PWM_GPIO_M2 5
#define PWM_GPIO_M3 21
#define PWM_GPIO_M4 12

static const int motor_gpio[MOTOR_COUNT] = {
    PWM_GPIO_M1,
    PWM_GPIO_M2,
    PWM_GPIO_M3,
    PWM_GPIO_M4
};

static const ledc_channel_t motor_channel[MOTOR_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

float motor_pwm[MOTOR_COUNT];   // 0–100 %

/* ------------------------------------------------------------ */

void motor_controller_init(void)
{
    /* Timer configuration */
    ledc_timer_config_t timer_config = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    /* Channel configuration */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num       = motor_gpio[i],
            .speed_mode     = PWM_MODE,
            .channel        = motor_channel[i],
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = PWM_TIMER,
            .duty           = 0,
            .hpoint         = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }
}

/* ------------------------------------------------------------ */

static inline float clamp(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

/* ------------------------------------------------------------ */

void motor_set_speed_percent(void)
{
    const uint32_t max_duty = (1 << 12) - 1;  // 4095 for 12-bit

    for (int i = 0; i < MOTOR_COUNT; i++) {

        motor_pwm[i] = clamp(motor_pwm[i], 0.0f, 100.0f);

        uint32_t duty = (uint32_t)((motor_pwm[i] / 100.0f) * max_duty);

        ESP_ERROR_CHECK(
            ledc_set_duty(PWM_MODE, motor_channel[i], duty)
        );

        ESP_ERROR_CHECK(
            ledc_update_duty(PWM_MODE, motor_channel[i])
        );
    }
}


void motor_controller(float throttle, float* rotation_rate_output)
{
    motor_pwm[0] = throttle*100.f; // M1
    motor_pwm[1] = throttle*100.f; // M2
    motor_pwm[2] = throttle*100.f; // M3
    motor_pwm[3] = throttle*100.f; // M4

    motor_set_speed_percent();
}

