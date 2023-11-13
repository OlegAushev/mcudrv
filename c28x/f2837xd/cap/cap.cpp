#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/cap/cap.h>


namespace mcu {


namespace cap {


const uint32_t impl::cap_bases[6] = {ECAP1_BASE, ECAP2_BASE, ECAP3_BASE, ECAP4_BASE, ECAP5_BASE, ECAP6_BASE};
const XBAR_InputNum impl::cap_xbar_inputs[6] = {XBAR_INPUT7, XBAR_INPUT8, XBAR_INPUT9, XBAR_INPUT10, XBAR_INPUT11, XBAR_INPUT12};
const uint32_t impl::cap_pie_int_nums[6] = {INT_ECAP1, INT_ECAP2, INT_ECAP3, INT_ECAP4, INT_ECAP5, INT_ECAP6};

void (*Module::_on_interrupt_callbacks[peripheral_count])(Module*, uint16_t);


Module::Module(Peripheral peripheral, const gpio::Config& pin_config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::cap_bases[peripheral.underlying_value()],
                  impl::cap_xbar_inputs[peripheral.underlying_value()],
                  impl::cap_pie_int_nums[peripheral.underlying_value()])
        , _pin(pin_config) {
    XBAR_setInputPin(_module.xbar_input, _pin.no());

    ECAP_disableInterrupt(_module.base,
                         (ECAP_ISR_SOURCE_CAPTURE_EVENT_1
                        | ECAP_ISR_SOURCE_CAPTURE_EVENT_2
                        | ECAP_ISR_SOURCE_CAPTURE_EVENT_3
                        | ECAP_ISR_SOURCE_CAPTURE_EVENT_4
                        | ECAP_ISR_SOURCE_COUNTER_OVERFLOW
                        | ECAP_ISR_SOURCE_COUNTER_PERIOD
                        | ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(_module.base,
                       (ECAP_ISR_SOURCE_CAPTURE_EVENT_1
                      | ECAP_ISR_SOURCE_CAPTURE_EVENT_2
                      | ECAP_ISR_SOURCE_CAPTURE_EVENT_3
                      | ECAP_ISR_SOURCE_CAPTURE_EVENT_4
                      | ECAP_ISR_SOURCE_COUNTER_OVERFLOW
                      | ECAP_ISR_SOURCE_COUNTER_PERIOD
                      | ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_disableTimeStampCapture(_module.base);
    ECAP_stopCounter(_module.base);
    ECAP_enableCaptureMode(_module.base);

    ECAP_setCaptureMode(_module.base, ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_2);

    ECAP_setEventPolarity(_module.base, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(_module.base, ECAP_EVENT_2, ECAP_EVNT_FALLING_EDGE);

    ECAP_enableCounterResetOnEvent(_module.base, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(_module.base, ECAP_EVENT_2);

    ECAP_setSyncOutMode(_module.base, ECAP_SYNC_OUT_DISABLED);
    ECAP_startCounter(_module.base);
    ECAP_enableTimeStampCapture(_module.base);
    ECAP_reArm(_module.base);
}


void Module::register_interrupt_handler(void (*handler)(void)) {
    Interrupt_register(_module.pie_int_num, handler);
    ECAP_enableInterrupt(_module.base, ECAP_ISR_SOURCE_CAPTURE_EVENT_1);
    ECAP_enableInterrupt(_module.base, ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
    ECAP_enableInterrupt(_module.base, ECAP_ISR_SOURCE_COUNTER_OVERFLOW);
}


void Module::register_interrupt_callback(void (*callback)(Module*, uint16_t)) {
    switch (_peripheral.native_value()) {
    case Peripheral::cap1:
        register_interrupt_handler(on_interrupt<Peripheral::cap1>);
        break;
    case Peripheral::cap2:
        register_interrupt_handler(on_interrupt<Peripheral::cap2>);
        break;
    case Peripheral::cap3:
        register_interrupt_handler(on_interrupt<Peripheral::cap3>);
        break;
    case Peripheral::cap4:
        register_interrupt_handler(on_interrupt<Peripheral::cap4>);
        break;
    case Peripheral::cap5:
        register_interrupt_handler(on_interrupt<Peripheral::cap5>);
        break;
    case Peripheral::cap6:
        register_interrupt_handler(on_interrupt<Peripheral::cap6>);
        break;
    }
    _on_interrupt_callbacks[_peripheral.underlying_value()] = callback;
}


} // namespace cap


} // namespace mcu


#endif
