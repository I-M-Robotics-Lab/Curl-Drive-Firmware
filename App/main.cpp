#include "common_inc.hpp"
#include "configurations.hpp"

static volatile bool g_drv_fault_irq = false;

Controller controller;
EncoderAS5047P encoder;
Driver8323s    drv;

void Main() {
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);

    pwr::init();
    temp::init();

    Controller::Config cfg{};
    controller.configure(cfg);

    int ok = drv.init();
    encoder.init();
    controller.init();

    usb::println("line echo ready");
    led::startToggle(30);
    HAL_Delay(500);
    led::stopToggle();

    led::setLed(!ok ? 1 : 0);
    HAL_Delay(1000);

    controller.arm();

    for (;;) {

        if (g_drv_fault_irq) {
            g_drv_fault_irq = false;
            //controller.disarm();
            usb::println("[DRV8323S] nFAULT asserted — check: curl-drive driver status");
        }

        //foc::status.Vd = 10;
        //foc::status.Vq = 0;

        //usb::println("No LUT = ", controller.status().raw_mechAng, ", LUT = ", controller.status().curr_mechAng);
        //usb::println("Angle = ", controller.status().curr_pos_raw, ", Angle1 = ", controller.status().curr_pos);
        usb::println("Id = ", foc::status.Id, ", Iq = ", foc::status.Iq);
        //usb::println("cA = ", foc::status.cA, ", cB = ", foc::status.cB, ", cC = ", foc::status.cC);
        //usb::println("fv = ", controller.status().curr_vel, ", tv = ", controller.status().TarVel);
        //usb::println("rev = ", controller.status().revolution_count, ", mech = ", controller.status().curr_mechAng, ", pos = ", controller.status().curr_pos);
        //usb::println("pos =", controller.status().curr_pos, ", tarpos =", controller.status().TarPos);

        cli_poll();
        HAL_Delay(20);
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == GPIO_PIN_3) {
        g_drv_fault_irq = true;
    }
}

