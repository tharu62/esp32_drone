#include "motor_controller.h"
// #include "driver/mcpwm_prelude.h"
// #include "esp_err.h"

#define MOTOR_COUNT 4

#define PWM_GPIO_M1 33
#define PWM_GPIO_M2 34
#define PWM_GPIO_M3 35
#define PWM_GPIO_M4 36

#define PWM_FREQ_HZ  20000      // 20 kHz for MOSFETs
#define TIMER_RES_HZ 1000000    // 1 MHz resolution

#define MAX_RES 360.0f          // Max control authority

#define A 1.f
#define B 1.f
#define C -1.f
#define D 1.f
#define E 1.f
#define F -1.f
#define G -1.f
#define H -1.f

static const int motor_gpio[MOTOR_COUNT] = {
    PWM_GPIO_M1,
    PWM_GPIO_M2,
    PWM_GPIO_M3,
    PWM_GPIO_M4
};

/* MCPWM handles */
static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t oper;
static mcpwm_cmpr_handle_t comparator[MOTOR_COUNT];
static mcpwm_gen_handle_t generator[MOTOR_COUNT];

/* Motor commands */
float motor_pwm[MOTOR_COUNT];

/* ------------------------------------------------------------ */

void motor_controller_init(void)
{
    /* Timer */
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RES_HZ,
        .period_ticks = TIMER_RES_HZ / PWM_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    /* Operator */
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* Create comparators & generators */
    for (int i = 0; i < MOTOR_COUNT; i++) {

        mcpwm_comparator_config_t cmp_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &comparator[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], 0));

        mcpwm_generator_config_t gen_config = {
            .gen_gpio_num = motor_gpio[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator[i]));

        /* PWM logic:
           - HIGH at timer start
           - LOW when counter reaches compare value
        */
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                comparator[i],
                MCPWM_GEN_ACTION_LOW)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
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
    uint32_t period = TIMER_RES_HZ / PWM_FREQ_HZ;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        
        clamp(motor_pwm[i], 0.0f, 100.0f);
        uint32_t compare = (uint32_t)((motor_pwm[i] / 100.0f) * period);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], compare));
    }
}

/* ------------------------------------------------------------ */

void motor_controller(float throttle, float *angle_error)
{
    float roll_error  = angle_error[0];
    float pitch_error = angle_error[1];

    /* X-configuration quad mixing */
    motor_pwm[0] = throttle + (((A*roll_error + B*pitch_error)/720.f) + 1) / 2; // Front Left
    motor_pwm[1] = throttle + (((C*roll_error + D*pitch_error)/720.f) + 1) / 2; // Front Right
    motor_pwm[2] = throttle + (((E*roll_error + F*pitch_error)/720.f) + 1) / 2; // Rear Right
    motor_pwm[3] = throttle + (((G*roll_error + H*pitch_error)/720.f) + 1) / 2; // Rear Left

    motor_set_speed_percent();
}
