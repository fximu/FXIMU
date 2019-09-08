#include "fximu.h"
#include "ros_fximu.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

char loginfo_buffer[128];

void ConfigureUART(void) {

    // enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // enable UART0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // configure serial port
    ConfigureUART();

    // init ports, hardware reset of sensor.
    init_system();

    // init i2c
    init_I2C2();

    // init interrupts
    init_interrupts();

    // initialize sensors, 50hz for calibration
    init_sensors(AFSR_4G, GFSR_250PS, ODR_50HZ);

    while(1) {
        if(data_ready) {
            sprintf(loginfo_buffer, "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", accelRD.x, accelRD.y, accelRD.z, gyroRD.x, gyroRD.y, gyroRD.z, magRD.x, magRD.y, magRD.z);
            UARTprintf(loginfo_buffer);
            data_ready = false;
        }
    }

}
