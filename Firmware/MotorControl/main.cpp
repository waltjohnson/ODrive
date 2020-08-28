
#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"

#include "usart.h"
#include "freertos_vars.h"
#include "usb_device.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>
#include <communication/interface_can.hpp>

osSemaphoreId sem_usb_irq;
osSemaphoreId sem_uart_dma;
osSemaphoreId sem_usb_rx;
osSemaphoreId sem_usb_tx;
osSemaphoreId sem_can;

#if defined(STM32F405xx)
// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script


ODriveCAN::Config_t can_config;
ODriveCAN *odCAN = nullptr;
ODrive odrv{};


ConfigManager config_manager;

static bool config_read_all() {
    bool success = board_read_config() &&
           config_manager.read(&odrv.config_) &&
           config_manager.read(&can_config);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.read(&encoders[i].config_) &&
                  config_manager.read(&axes[i].sensorless_estimator_.config_) &&
                  config_manager.read(&axes[i].controller_.config_) &&
                  config_manager.read(&axes[i].trap_traj_.config_) &&
                  config_manager.read(&axes[i].min_endstop_.config_) &&
                  config_manager.read(&axes[i].max_endstop_.config_) &&
                  config_manager.read(&motors[i].config_) &&
                  config_manager.read(&fet_thermistors[i].config_) &&
                  config_manager.read(&axes[i].motor_thermistor_.config_) &&
                  config_manager.read(&axes[i].config_);
    }
    return success;
}

static bool config_write_all() {
    bool success = board_write_config() &&
           config_manager.write(&odrv.config_) &&
           config_manager.write(&can_config);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.write(&encoders[i].config_) &&
                  config_manager.write(&axes[i].sensorless_estimator_.config_) &&
                  config_manager.write(&axes[i].controller_.config_) &&
                  config_manager.write(&axes[i].trap_traj_.config_) &&
                  config_manager.write(&axes[i].min_endstop_.config_) &&
                  config_manager.write(&axes[i].max_endstop_.config_) &&
                  config_manager.write(&motors[i].config_) &&
                  config_manager.write(&fet_thermistors[i].config_) &&
                  config_manager.write(&axes[i].motor_thermistor_.config_) &&
                  config_manager.write(&axes[i].config_);
    }
    return success;
}

static void config_clear_all() {
    odrv.config_ = {};
    can_config = {};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        encoders[i].config_ = {};
        axes[i].sensorless_estimator_.config_ = {};
        axes[i].controller_.config_ = {};
        axes[i].controller_.config_.load_encoder_axis = i;
        axes[i].trap_traj_.config_ = {};
        axes[i].min_endstop_.config_ = {};
        axes[i].max_endstop_.config_ = {};
        motors[i].config_ = {};
        fet_thermistors[i].config_ = {};
        axes[i].motor_thermistor_.config_ = {};
        axes[i].clear_config();
    }
}

static bool config_apply_all() {
    bool success = true;
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = encoders[i].apply_config(motors[i].config_.motor_type)
               && axes[i].controller_.apply_config()
               && axes[i].min_endstop_.apply_config()
               && axes[i].max_endstop_.apply_config()
               && motors[i].apply_config()
               && axes[i].apply_config();
    }
    return success;
}

void ODrive::save_configuration(void) {
    size_t config_size = 0;
    bool success = config_manager.prepare_store()
                && config_write_all()
                && config_manager.start_store(&config_size)
                && config_write_all()
                && config_manager.finish_store();
    if (success) {
        user_config_loaded_ = config_size;
    } else {
        printf("saving configuration failed\r\n");
        osDelay(5);
    }
}

void ODrive::erase_configuration(void) {
    NVM_erase();

    // FIXME: this reboot is a workaround because we don't want the next save_configuration
    // to write back the old configuration from RAM to NVM. The proper action would
    // be to reset the values in RAM to default. However right now that's not
    // practical because several startup actions depend on the config. The
    // other problem is that the stack overflows if we reset to default here.
    NVIC_SystemReset();
}

void ODrive::enter_dfu_mode() {
    if ((hw_version_major_ == 3) && (hw_version_minor_ >= 5)) {
        __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        _reboot_cookie = 0xDEADBEEF;
        NVIC_SystemReset();
    } else {
        /*
        * DFU mode is only allowed on board version >= 3.5 because it can burn
        * the brake resistor FETs on older boards.
        * If you really want to use it on an older board, add 3.3k pull-down resistors
        * to the AUX_L and AUX_H signals and _only then_ uncomment these lines.
        */
        //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        //_reboot_cookie = 0xDEADFE75;
        //NVIC_SystemReset();
    }
}

void ODrive::clear_errors() {
    for (auto& axis: axes) {
        axis.motor_.error_ = Motor::ERROR_NONE;
        axis.controller_.error_ = Controller::ERROR_NONE;
        axis.sensorless_estimator_.error_ = SensorlessEstimator::ERROR_NONE;
        axis.encoder_.error_ = Encoder::ERROR_NONE;
        axis.encoder_.spi_error_rate_ = 0.0f;
        axis.error_ = Axis::ERROR_NONE;
    }
    error_ = ERROR_NONE;
}

extern "C" {

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for(auto& axis: axes){
        axis.motor_.disarm();
    }
    safety_critical_disarm_brake_resistor();
    for (;;); // TODO: safe action
}

void vApplicationIdleHook(void) {
    if (odrv.system_stats_.fully_booted) {
        odrv.system_stats_.uptime = xTaskGetTickCount();
        odrv.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
        uint32_t min_stack_space[AXIS_COUNT];
        std::transform(axes.begin(), axes.end(), std::begin(min_stack_space), [](auto& axis) { return uxTaskGetStackHighWaterMark(axis.thread_id_) * sizeof(StackType_t); });
        odrv.system_stats_.min_stack_space_axis = *std::min_element(std::begin(min_stack_space), std::end(min_stack_space));
        odrv.system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_can = uxTaskGetStackHighWaterMark(odCAN->thread_id_) * sizeof(StackType_t);

        // Actual usage, in bytes, so we don't have to math
        odrv.system_stats_.stack_usage_axis = axes[0].stack_size_ - odrv.system_stats_.min_stack_space_axis;
        odrv.system_stats_.stack_usage_usb = stack_size_usb_thread - odrv.system_stats_.min_stack_space_usb;
        odrv.system_stats_.stack_usage_uart = stack_size_uart_thread - odrv.system_stats_.min_stack_space_uart;
        odrv.system_stats_.stack_usage_startup = stack_size_default_task - odrv.system_stats_.min_stack_space_startup;
        odrv.system_stats_.stack_usage_can = odCAN->stack_size_ - odrv.system_stats_.min_stack_space_can;
    }
}

}

/**
 * @brief Runs system-level checks that need to be as real-time as possible.
 * 
 * This function is called after every current measurement of every motor.
 * It should finish as quickly as possible.
 */
void ODrive::do_fast_checks() {
    if (!(vbus_voltage >= config_.dc_bus_undervoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_UNDER_VOLTAGE);
    if (!(vbus_voltage <= config_.dc_bus_overvoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_OVER_VOLTAGE);
}

/**
 * @brief Floats all power phases on the system (all motors and brake resistors).
 *
 * This should be called if a system level exception ocurred that makes it
 * unsafe to run power through the system in general.
 */
void ODrive::disarm_with_error(Error error) {
    CRITICAL_SECTION() {
        for (auto& axis: axes) {
            axis.motor_.disarm_with_error(Motor::ERROR_SYSTEM_LEVEL);
        }
        safety_critical_disarm_brake_resistor();
        error_ |= error;
    }
}

/**
 * @brief Runs the periodic sampling tasks
 * 
 * All components that need to sample real-world data should do it in this
 * function as it runs on a high interrupt priority and provides lowest possible
 * timing jitter.
 * 
 * All function called from this function should adhere to the following rules:
 *  - Try to use the same number of CPU cycles in every iteration.
 *    (reason: Tasks that run later in the function still want lowest possible timing jitter)
 *  - Use as few cycles as possible.
 *    (reason: The interrupt blocks other important interrupts (TODO: which ones?))
 *  - Not call any FreeRTOS functions.
 *    (reason: The interrupt priority is higher than the max allowed priority for syscalls)
 * 
 * Time consuming and undeterministic logic/arithmetic should live on
 * control_loop_cb() instead.
 */
void ODrive::sampling_cb() {
    n_evt_sampling_++;

    MEASURE_TIME(task_times_.sampling) {
        for (auto& axis: axes) {
            axis.encoder_.sample_now();
        }
    }
}

/**
 * @brief Runs the periodic control loop.
 * 
 * This function is executed in a low priority interrupt context and is allowed
 * to call CMSIS functions.
 * 
 * Yet it runs at a higher priority than communication workloads.
 * 
 * @param update_cnt: The true count of update events (wrapping around at 16
 *        bits). This is used for timestamp calculation in the face of
 *        potentially missed timer update interrupts. Therefore this counter
 *        must not rely on any interrupts.
 */
void ODrive::control_loop_cb(uint32_t timestamp) {
    last_update_timestamp_ = timestamp;
    n_evt_control_loop_++;

    // TODO: use a configurable component list for most of the following things

    MEASURE_TIME(task_times_.control_loop_misc) {
        uart_poll();
        odrv.oscilloscope_.update();
    }

    MEASURE_TIME(task_times_.control_loop_checks) {
        for (auto& axis: axes) {
            // look for errors at axis level and also all subcomponents
            bool checks_ok = axis.do_checks(timestamp);

            // make sure the watchdog is being fed. 
            bool watchdog_ok = axis.watchdog_check();

            if (!checks_ok || !watchdog_ok) {
                axis.motor_.disarm();
            }
        }
    }

    for (auto& axis: axes) {
        // Sub-components should use set_error which will propegate to this error_
        MEASURE_TIME(axis.task_times_.thermistor_update) {
            for (ThermistorCurrentLimiter* thermistor : axis.thermistors_) {
                thermistor->update();
            }
        }

        MEASURE_TIME(axis.task_times_.encoder_update)
            axis.encoder_.update();

        MEASURE_TIME(axis.task_times_.sensorless_estimator_update)
            axis.sensorless_estimator_.update();

        MEASURE_TIME(axis.task_times_.endstop_update) {
            axis.min_endstop_.update();
            axis.max_endstop_.update();
        }

        MEASURE_TIME(axis.task_times_.can_heartbeat)
            odCAN->send_heartbeat(&axis);

        MEASURE_TIME(axis.task_times_.controller_update)
            axis.controller_.update(); // uses position and velocity from encoder

        MEASURE_TIME(axis.task_times_.open_loop_controller_update)
            axis.open_loop_controller_.update(timestamp);

        MEASURE_TIME(axis.task_times_.async_estimator_update)
            axis.async_estimator_.update(timestamp);

        MEASURE_TIME(axis.task_times_.motor_update)
            axis.motor_.update(); // uses torque from controller and phase_vel from encoder

        MEASURE_TIME(axis.task_times_.current_controller_update)
            axis.motor_.current_control_.update(timestamp); // uses the output of controller_ or open_loop_contoller_ and encoder_ or sensorless_estimator_ or async_estimator_
    }
}


/** @brief For diagnostics only */
uint32_t ODrive::get_interrupt_status(int32_t irqn) {
    if ((irqn < -14) || (irqn >= 240)) {
        return 0xffffffff;
    }

    uint8_t priority = (irqn < -12)
        ? 0 // hard fault and NMI always have maximum priority
        : NVIC_GetPriority((IRQn_Type)irqn);
    uint32_t counter = GET_IRQ_COUNTER((IRQn_Type)irqn);
    bool is_enabled = (irqn < 0)
        ? true // processor interrupt vectors are always enabled
        : NVIC->ISER[(((uint32_t)(int32_t)irqn) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)(int32_t)irqn) & 0x1FUL));
    
    return priority | ((counter & 0x7ffffff) << 8) | (is_enabled ? 0x80000000 : 0);
}

/** @brief For diagnostics only */
uint32_t ODrive::get_dma_status(uint8_t stream_num) {
    DMA_Stream_TypeDef* streams[] = {
        DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
    };
    if (stream_num >= 16) {
        return 0xffffffff;
    }
    DMA_Stream_TypeDef* stream = streams[stream_num];
    bool is_reset = (stream->CR == 0x00000000)
                 && (stream->NDTR == 0x00000000)
                 && (stream->PAR == 0x00000000)
                 && (stream->M0AR == 0x00000000)
                 && (stream->M1AR == 0x00000000)
                 && (stream->FCR == 0x00000021);
    uint8_t channel = ((stream->CR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);
    uint8_t priority = ((stream->CR & DMA_SxCR_PL_Msk) >> DMA_SxCR_PL_Pos);
    return (is_reset ? 0 : 0x80000000) | ((channel & 0x7) << 2) | (priority & 0x3);
}


/**
 * @brief Main thread started from main().
 */
static void rtos_main(void*) {
    // Init USB device
    MX_USB_DEVICE_Init();


    // Start ADC for temperature measurements and user measurements
    start_general_purpose_adc();

    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Start pwm-in compare modules
    // must happen after communication is initialized
    pwm0_input.init();

    // Try to initialized gate drivers for fault-free startup.
    // If this does not succeed, a fault will be raised and the idle loop will
    // periodically attempt to reinit the gate driver.
    for(auto& axis: axes){
        axis.motor_.setup();
    }

    for(auto& axis: axes){
        axis.encoder_.setup();
    }

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }

    start_analog_thread();

    odrv.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal).
    vTaskDelete(defaultTaskHandle);
}

/**
 * @brief Carries out early startup tasks that need to run before any static
 * initializers.
 * This function gets called from the startup assembly code.
 */
extern "C" void early_start_checks(void) {
    if(_reboot_cookie == 0xDEADFE75) {
        /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
        * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
        * FETs and obliterating them unless external 3.3k pull-down resistors are
        * present. Pull-downs are only present on ODrive 3.5 or newer.
        * On older boards we disable DFU by default but if the user insists
        * there's only one thing left that might save it: time.
        * The brake resistor gate driver needs a certain 10V supply (GVDD) to
        * make it work. This voltage is supplied by the motor gate drivers which get
        * disabled at system reset. So over time GVDD voltage _should_ below
        * dangerous levels. This is completely handwavy and should not be relied on
        * so you are on your own on if you ignore this warning.
        *
        * This loop takes 5 cycles per iteration and at this point the system runs
        * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
        */
        for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
            __NOP();
        }
        _reboot_cookie = 0xDEADBEEF;
    }

    /* We could jump to the bootloader directly on demand without rebooting
    but that requires us to reset several peripherals and interrupts for it
    to function correctly. Therefore it's easier to just reset the entire chip. */
    if(_reboot_cookie == 0xDEADBEEF) {
        _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger
        __set_MSP((uintptr_t)&_estack);
        // http://www.st.com/content/ccc/resource/technical/document/application_note/6a/17/92/02/58/98/45/0c/CD00264379.pdf/files/CD00264379.pdf
        void (*builtin_bootloader)(void) = (void (*)(void))(*((uint32_t *)0x1FFF0004));
        builtin_bootloader();
    }

    /* The bootloader might fail to properly clean up after itself,
    so if we're not sure that the system is in a clean state we
    just reset it again */
    if(_reboot_cookie != 42) {
        _reboot_cookie = 42;
        NVIC_SystemReset();
    }
}

/**
 * @brief Main entry point called from assembly startup code.
 */
extern "C" int main(void) {
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    serial_number = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);

    uint64_t val = serial_number;
    for (size_t i = 0; i < 12; ++i) {
        serial_number_str[i] = "0123456789ABCDEF"[(val >> (48-4)) & 0xf];
        val <<= 4;
    }
    serial_number_str[12] = 0;

    // Init low level system functions (clocks, flash interface)
    system_init();

    // Load configuration from NVM. This needs to happen after system_init()
    // since the flash interface must be initialized and before board_init()
    // since board initialization can depend on the config.
    size_t config_size = 0;
    bool success = config_manager.start_load()
            && config_read_all()
            && config_manager.finish_load(&config_size)
            && config_apply_all();
    if (success) {
        odrv.user_config_loaded_ = config_size;
    } else {
        config_clear_all();
        config_apply_all();
    }

    odrv.misconfigured_ = odrv.misconfigured_
            || (odrv.config_.enable_uart0 && !uart0)
            || (odrv.config_.enable_uart1 && !uart1)
            || (odrv.config_.enable_uart2 && !uart2);

    // Init board-specific peripherals
    if (!board_init()) {
        for (;;); // TODO: handle properly
    }

    // Init GPIOs according to their configured mode
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        // Skip unavailable GPIOs
        if (!get_gpio(i)) {
            continue;
        }

        ODriveIntf::GpioMode mode = odrv.config_.gpio_modes[i];

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = get_gpio(i).pin_mask_;

        // Set Alternate Function setting for this GPIO mode
        if (mode == ODriveIntf::GPIO_MODE_DIGITAL || mode == ODriveIntf::GPIO_MODE_ANALOG_IN) {
            GPIO_InitStruct.Alternate = 0;
        } else {
            auto it = std::find_if(
                    alternate_functions[i].begin(), alternate_functions[i].end(),
                    [mode](auto a) { return a.mode == mode; });

            if (it == alternate_functions[i].end()) {
                odrv.misconfigured_ = true; // this GPIO doesn't support the selected mode
                continue;
            }
            GPIO_InitStruct.Alternate = it->alternate_function;
        }

        switch (mode) {
            case ODriveIntf::GPIO_MODE_DIGITAL: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ANALOG_IN: {
                GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
            } break;
            case ODriveIntf::GPIO_MODE_UART0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART1: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart1) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART2: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart2) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_CAN0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_can0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_I2C0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_i2c0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            //case ODriveIntf::GPIO_MODE_SPI0: { // TODO
            //} break;
            case ODriveIntf::GPIO_MODE_PWM0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC1: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC2: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            default: {
                odrv.misconfigured_ = true;
                continue;
            }
        }

        HAL_GPIO_Init(get_gpio(i).port_, &GPIO_InitStruct);
    }

    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
    osSemaphoreWait(sem_usb_irq, 0);

    // Create a semaphore for UART DMA and remove a token
    osSemaphoreDef(sem_uart_dma);
    sem_uart_dma = osSemaphoreCreate(osSemaphore(sem_uart_dma), 1);

    // Create a semaphore for USB RX
    osSemaphoreDef(sem_usb_rx);
    sem_usb_rx = osSemaphoreCreate(osSemaphore(sem_usb_rx), 1);
    osSemaphoreWait(sem_usb_rx, 0);  // Remove a token.

    // Create a semaphore for USB TX
    osSemaphoreDef(sem_usb_tx);
    sem_usb_tx = osSemaphoreCreate(osSemaphore(sem_usb_tx), 1);

    osSemaphoreDef(sem_can);
    sem_can = osSemaphoreCreate(osSemaphore(sem_can), 1);
    osSemaphoreWait(sem_can, 0);

    // Construct all objects.
    odCAN = new ODriveCAN(can_config, &hcan1);

    // Create main thread
    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Start scheduler
    osKernelStart();
    
    for (;;);
}
