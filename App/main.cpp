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




// Define board role: set to true for transmitter board, false for receiver board
#define IS_TRANSMITTER_BOARD true	// Change this for each board

// Global variables for test
bool message_received = false;
uint32_t received_id;
uint8_t received_data[64];
uint32_t received_length;
uint32_t message_count = 0;
uint8_t test_data[64];
// Global callbacks for FDCAN
void fdcan_rx_callback(uint32_t id, const uint8_t* data, uint32_t length) {
    message_received = true;
    received_id = id;
    received_length = length;
    memcpy(received_data, data, length);
}

void main_app()
{
	// Reset test flag
	message_received = false;
    message_count = 0;

    // FDCAN Test Code
    FDCANConfig config;
    if (IS_TRANSMITTER_BOARD) {
        // Transmitter accepts acknowledgments on 0x101
        config = {
            .filter_id1 = 0x101,
            .filter_id2 = 0x101,
            .filter_type = FDCAN_FILTER_MASK,
            .brs_on = false
        };
    } else {
        // Receiver accepts data messages on 0x100
        config = {
            .filter_id1 = 0x100,
            .filter_id2 = 0x100,
            .filter_type = FDCAN_FILTER_MASK,
            .brs_on = false
        };
    }

    if (!g_fdcan_comm.init(config, fdcan_rx_callback)) {
        // Initialization failed - could add error indication
        return;
    }

    if (IS_TRANSMITTER_BOARD) {
        // Transmitter board: send messages periodically
        while (1) {
            // Test data - 64 bytes, declare volatile to prevent compiler optimization
            for (uint8_t i = 0; i < 64; i++) {
            	  test_data[i] = (uint8_t)message_count;
            }

            if (g_fdcan_comm.transmit(0x100, (uint8_t*)test_data, 64)) {
                message_count++;
            }

            // Wait 1 second before next transmission
            HAL_Delay(1000);

            // Poll for error status continuously during wait
            FDCANError error_status = g_fdcan_comm.getErrorStatus();
            switch (error_status) {
                case FDCAN_ERR_WARNING:
                    // Error warning - TEC/REC > 96
                    // Add warning handling code
                    break;
                case FDCAN_ERR_PASSIVE:
                    // Error passive - TEC/REC > 127
                    // Add passive handling code
                    break;
                case FDCAN_BUS_OFF:
                    // Bus-off - TEC > 255, requires recovery
                    g_fdcan_comm.recoverFromBusOff();
                    break;
                case FDCAN_ERR_NONE:
                default:
                    // No error
                    break;
            }

            // Check for received messages (acknowledgments)
            if (message_received) {
                // Received acknowledgment from receiver board
                message_received = false;
                // Could add success indication here
            }
        }
    } else {
        // Receiver board: wait for messages and respond
        while (1) {
            if (message_received) {
                // Check if received message matches expected
                if (received_id == 0x100 && received_length == 64) {
                    // Send acknowledgment back
                    uint8_t ack_data[64] = {0};  // Initialize to zeros
                    ack_data[0] = 0xFF;  // Acknowledgment marker
                    ack_data[1] = received_data[0];  // Echo the counter
                    g_fdcan_comm.transmit(0x101, ack_data, 64);
                }
                message_received = false;
            }

            // Poll for error status continuously
            FDCANError error_status = g_fdcan_comm.getErrorStatus();
            switch (error_status) {
                case FDCAN_ERR_WARNING:
                    // Error warning - TEC/REC > 96
                    // Add warning handling code
                    break;
                case FDCAN_ERR_PASSIVE:
                    // Error passive - TEC/REC > 127
                    // Add passive handling code
                    break;
                case FDCAN_BUS_OFF:
                    // Bus-off - TEC > 255, requires recovery
                    g_fdcan_comm.recoverFromBusOff();
                    break;
                case FDCAN_ERR_NONE:
                default:
                    // No error
                    break;
            }

            // Small delay to prevent complete CPU hogging
            HAL_Delay(1);  // 1ms delay for minimal CPU usage
        }
    }

    // Deinit (unreachable in current setup)
    //g_fdcan_comm.deinit();
}

