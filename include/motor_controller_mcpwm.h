#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "state.h"

void motor_controller_init(void);

void motor_controller(State *ds);

void motor_off(void);