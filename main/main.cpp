#include "uart_bridge.h"
#include "kinematics.h"
#include "pca9685_driver.h"
#include "ina219_driver.h"

extern "C" void app_main(void) {
    UartBridge uart;
    Kinematics kinematics;
    Pca9685Driver servos;
    Ina219Driver battery;

    uart.init();
    kinematics.init();
    servos.init();
    battery.init();
}
