#include "common_inc.hpp"
#include "configurations.hpp"

static volatile bool g_drv_fault_irq = false;

Controller controller;
EncoderAS5047P encoder;
Driver8323s    drv;

void Main() {
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);

    vbat::init();
    temp::init();

    Controller::Config cfg{};
    controller.configure(cfg);

    int ok = drv.init();
    encoder.init();
    controller.init();

    usb::println("line echo ready");
    led::startToggle(20);
    HAL_Delay(500);
    led::stopToggle();

    //controller.startCalibrate();
    led::setLed(!ok ? 1 : 0);
    HAL_Delay(1000);

    controller.arm();

    for (;;) {

        if (g_drv_fault_irq) {
            g_drv_fault_irq = false;
            //controller.setCoast();
            usb::println("[DRV8323S] nFAULT asserted — check: curl-drive driver status");
        }

        //foc::status.Vd = 10;
        //foc::status.Vq = 0;

        //usb::println(", Iq = ", foc::status.Iq);
        usb::println("cA = ", foc::status.cA, ", cB = ", foc::status.cB, ", cC = ", foc::status.cC);
        //foc::status.phaseA = 9;
        //foc::status.phaseA = -4.5;
        //foc::status.phaseA = -4.5;

        controller.execute();

        cli_poll();
        HAL_Delay(50);
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == GPIO_PIN_3) {
        g_drv_fault_irq = true;
    }
}

