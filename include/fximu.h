#ifndef fximu_h
#define fximu_h

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <ros.h>
#include <ros/time.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"

uint32_t ui32SysClkFreq;

void init_system() {

    // record system clock value after clock initialization
    ui32SysClkFreq = MAP_SysCtlClockGet();

    // Peripheral A enable
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // sensor hard reset
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    MAP_SysCtlDelay(10000);

    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x00);
    MAP_SysCtlDelay(10000);

    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    MAP_SysCtlDelay(10000);



}

#ifdef __cplusplus
}
#endif

#endif