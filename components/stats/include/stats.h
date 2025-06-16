#ifndef STATS_H_
#define STATS_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define STATS_TASK_PRIO     3
#define STATS_TICKS         pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET   5

void stats_run(void);

#endif // STATS_H_
