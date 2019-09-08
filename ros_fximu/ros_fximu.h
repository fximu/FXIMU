#ifndef ros_fximu_h
#define ros_fximu_h

#define FXAS21002C_ADDRESS (0x21)
#define FXOS8700_ADDRESS (0x1F)

#include "fxas21002c.h"
#include "fxos8700cq.h"

tRawData gyroRD;
tRawData accelRD;
tRawData magRD;

volatile bool gyro_data_ready = false;
volatile bool accelmag_data_ready = false;

void init_I2C2(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

// initialize sensor devices
void init_sensors(tAccelRange tAFSR, tGyroRange tGFSR, tOutputDataRate aODR, tOutputDataRate gODR) {
    AGInit(FXOS8700_ADDRESS, tAFSR, aODR);
    GyroInit(FXAS21002C_ADDRESS, tGFSR, gODR);
}

// gyro interrupt service routine
void gyro_isr(void) {
    if(GPIOIntStatus(GPIO_PORTC_BASE, false) & GPIO_PIN_4) {
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
        gyro_data_ready = true;
    }
}

// accelmag interrupt service routine
void accelmag_isr(void) {
    if(GPIOIntStatus(GPIO_PORTE_BASE, false) & GPIO_PIN_2) {
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
        accelmag_data_ready = true;
    }
}


// initialize interrupt pin for gyro
void init_interrupts(void) {

    // gyro interrupt pin setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntRegister(GPIO_PORTC_BASE, gyro_isr);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

    // accelmag interrupt pin setup
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntRegister(GPIO_PORTE_BASE, accelmag_isr);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_2);

    // TODO: gyro interrupt should have higher priority. this case does not.
    IntPrioritySet(INT_GPIOC, 0x00);
    IntPrioritySet(INT_GPIOE, 0x01);


}

#endif