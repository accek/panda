#ifndef SAFETY_VOLKSWAGEN_COMMON_H
#define SAFETY_VOLKSWAGEN_COMMON_H

#include <stdint.h>

const uint16_t FLAG_VOLKSWAGEN_LONG_CONTROL = 1;

bool volkswagen_longitudinal = false;
bool volkswagen_set_button_prev = false;
bool volkswagen_resume_button_prev = false;
bool volkswagen_stock_acc_engaged = false;

#endif
