#ifndef __CORE1_H__
#define __CORE1_H__

#include <stdio.h>

#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "CommandParsers.h"
#include "Protocol.h"
#include "UartUtils.h"
#include "Utils.h"

extern ServoData servoData[ServoNum + 1];
extern DesiredSettings desiredSettings[ServoNum + 1];
extern float controlOutputArray[ServoNum + 1];
extern ServoLimits servoLimits[ServoNum + 1];

void core1_thread();

#endif