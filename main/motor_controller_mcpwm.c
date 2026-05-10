#include "motor_controller_mcpwm.h"

#define MOTOR_COUNT 4

#define PWM_FREQ_HZ 50
#define PWM_TIMER_RES_HZ 1000000
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

#define PWM_GPIO_M0 1
#define PWM_GPIO_M1 3
#define PWM_GPIO_M2 4
#define PWM_GPIO_M3 38

#define MINIMUM_THROTTLE 0.1f

static const int motor_gpio[MOTOR_COUNT] = {
    PWM_GPIO_M0,
    PWM_GPIO_M1,
    PWM_GPIO_M2,
    PWM_GPIO_M3
};

static mcpwm_cmpr_handle_t comparator[MOTOR_COUNT];
static mcpwm_oper_handle_t operators[MOTOR_COUNT / 2];
static mcpwm_gen_handle_t generators[MOTOR_COUNT];

static uint32_t period_ticks;

void motor_controller_init(void)
{
    // 1. Create timer
    mcpwm_timer_handle_t timer = NULL;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_TIMER_RES_HZ, // 1 MHz → 1 tick = 1 µs
        .period_ticks = PWM_TIMER_RES_HZ / PWM_FREQ_HZ,    // 10 ms (100 Hz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    period_ticks = PWM_TIMER_RES_HZ / PWM_FREQ_HZ;

    for (int op = 0; op < MOTOR_COUNT / 2; op++) {

        // 2. Create operator
        mcpwm_operator_config_t op_config = {
            .group_id = 0
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &operators[op]));

        // 3. Connect operator to timer
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[op], timer));

        for (int chan = 0; chan < 2; chan++) {
            int motor_id = op * 2 + chan;

            // 4. Create comparator
            mcpwm_comparator_config_t cmp_config = {
                .flags.update_cmp_on_tez = true // update at timer zero → synchronized
            };
            ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op], &cmp_config, &comparator[motor_id]));

            // 5. Create generator (PWM output)
            mcpwm_generator_config_t gen_config = {
                .gen_gpio_num = motor_gpio[motor_id]
            };
            ESP_ERROR_CHECK(mcpwm_new_generator(operators[op], &gen_config, &generators[motor_id]));

            // 6. Set PWM behavior
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                generators[motor_id],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                generators[motor_id],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[motor_id], MCPWM_GEN_ACTION_LOW)));

            // 7. Start at minimum throttle
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[motor_id], ESC_MIN_US));
        }
    }

    // 8. Enable + start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static inline float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void motor_controller(State *ds)
{
    float M[4];

    float roll  = ds->pid_output[0];
    float pitch = ds->pid_output[1];
    float yaw   = ds->d_angle[2];
    float throttle = ds->throttle;

    if(throttle > MINIMUM_THROTTLE){
        // motors with full pid controll
        M[0] = throttle - roll + pitch + yaw;
        M[1] = throttle + roll - pitch + yaw;
        M[2] = throttle + roll + pitch - yaw;
        M[3] = throttle - roll - pitch - yaw;

        // TEST MODE with throttle control only
        // M[0] = throttle;
        // M[1] = throttle;
        // M[2] = throttle;
        // M[3] = throttle;

    } else {
        // If throttle is very low, set all motors to idle (minimum throttle) 
        // to prevent motor stalling and ensure better control response when 
        // throttle is increased again.
        for (int i = 0; i < MOTOR_COUNT; i++) {
            M[i] = 0.1f;
        }
    }

    // Update all motors
    for (int i = 0; i < MOTOR_COUNT; i++) {

        M[i] = clampf(M[i], 0.0f, 0.99f);

        uint32_t pulse_us = ESC_MIN_US + (uint32_t)(M[i] * (ESC_MAX_US - ESC_MIN_US));

        mcpwm_comparator_set_compare_value(comparator[i], pulse_us);
    }

}

void motor_off()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        mcpwm_comparator_set_compare_value(comparator[i], ESC_MIN_US);
    }
}
