#pragma once


#include "../system/system.h"
#include "../gpio/gpio.h"


namespace mcu {

namespace cap {

SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    cap1,
    cap2,
    cap3,
    cap4,
    cap5,
    cap6
} SCOPED_ENUM_DECLARE_END(Peripheral)


const int peripheral_count = 6;


namespace impl {

struct Module {
    uint32_t base;
    XBAR_InputNum xbar_input;
    uint32_t pie_int_num;
    Module(uint32_t base_, XBAR_InputNum xbar_input_, uint32_t pie_int_num_)
            : base(base_), xbar_input(xbar_input_), pie_int_num(pie_int_num_) {}
};


extern const uint32_t cap_bases[6];
extern const XBAR_InputNum cap_xbar_inputs[6];
extern const uint32_t cap_pie_int_nums[6];

} // namespace impl


class Module : public emb::c28x::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    impl::Module _module;
    gpio::Input _pin;
public:
    Module(Peripheral peripheral, const gpio::Config& pin_config);
    Peripheral peripheral() const { return _peripheral; }
    uint32_t base() const { return _module.base; }
    const gpio::Input& pin() const { return _pin; }
    uint32_t counter() const { return ECAP_getTimeBaseCounter(_module.base); }
    uint32_t event_timestamp(ECAP_Events event) const { return ECAP_getEventTimeStamp(_module.base, event); }

    void rearm() { ECAP_reArm(_module.base); }

    void register_interrupt_handler(void (*handler)(void));
    void register_interrupt_callback(void (*callback)(Module*, uint16_t));
    void enable_interrupts() { Interrupt_enable(_module.pie_int_num); }
    void disable_interrupts() { Interrupt_disable(_module.pie_int_num); }
    uint16_t interrupt_source() const { return ECAP_getInterruptSource(_module.base); }

    void acknowledge_interrupt(uint16_t interrupt_source) {
        ECAP_clearInterrupt(_module.base, interrupt_source);
        ECAP_clearGlobalInterrupt(_module.base);
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
    }
protected:
    static void (*_on_interrupt_callbacks[peripheral_count])(Module*, uint16_t);

    template <Peripheral::enum_type Periph>
    static interrupt void on_interrupt() {
        Module* module = Module::instance(Periph);
        uint16_t interrupt_source = ECAP_getInterruptSource(module->base());

        _on_interrupt_callbacks[Periph](module, interrupt_source);

        module->acknowledge_interrupt(interrupt_source);
    }
};

} // namespace cap

} // namespace mcu

