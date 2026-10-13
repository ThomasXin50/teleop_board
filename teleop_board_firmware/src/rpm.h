#ifndef RPM_H
#define RPM_H

#include <stdint.h>

#include "config.h"

void rpm_init(void);
void rpm_read_all(float wheel_rpm[RPM_WHEEL_COUNT], uint32_t now_us);

#endif