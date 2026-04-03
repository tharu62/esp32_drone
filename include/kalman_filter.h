#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "state.h"

void reset_kalman_filter();
 
void kalman_filter(State *drone_state, float dt);

#endif // KALMAN_FILTER_H