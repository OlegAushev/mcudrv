#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/can/can.h>
#include <mcudrv/stm32/f4/chrono/chrono.h>


namespace mcu {
namespace can {


Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _rx_pin.initialize({
    .port = rx_pin_config.port,
    .pin = {
        .Pin = rx_pin_config.pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = rx_pin_config.af_selection
    },
    .actstate = emb::gpio::active_pin_state::high});

    _tx_pin.initialize({
        .port = tx_pin_config.port,
        .pin = {
            .Pin = tx_pin_config.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = tx_pin_config.af_selection
        },
        .actstate = emb::gpio::active_pin_state::high});

    _enable_clk(peripheral);
    _reg = impl::can_instances[static_cast<size_t>(_peripheral)];
    _handle.Instance = _reg;
    _handle.Init = config.hal_config;

    if(HAL_CAN_Init(&_handle) != HAL_OK) {
        fatal_error("CAN module initialization failed");
    }
}


RxMessageAttribute Module::register_rxmessage(CAN_FilterTypeDef& filter) {
    RxMessageAttribute attr = {};
    
    if (_filter_count >= max_fitler_count) {
        fatal_error("too many CAN Rx filters");
    }

    if (_peripheral == Peripheral::can1) {
        filter.FilterBank = _filter_count++;
    } else {
        filter.FilterBank = 14 + _filter_count++;
    }
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    attr.filter_idx = filter.FilterBank;
    attr.fifo = RxFifo(filter.FilterFIFOAssignment);

    if (HAL_CAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        fatal_error("CAN module Rx filter configuration failed");
    }

    return attr;
}


void Module::start() {
    clear_bit<uint32_t>(_reg->MCR, CAN_MCR_INRQ);
    mcu::chrono::Timeout start_timeout(std::chrono::milliseconds(2));
    while (bit_is_set<uint32_t>(_reg->MSR, CAN_MSR_INAK)) {
        if (start_timeout.expired()) {
             fatal_error("CAN module start failed");
        }
    }
}


void Module::stop() {
    set_bit<uint32_t>(_reg->MCR, CAN_MCR_INRQ);
    mcu::chrono::Timeout stop_timeout(std::chrono::milliseconds(2));
    while (bit_is_clear<uint32_t>(_reg->MSR, CAN_MSR_INAK)) {
        if (stop_timeout.expired()) {
             fatal_error("CAN module start failed");
        }
    }
}


DrvStatus Module::send(const can_frame& frame) {
    if (mailbox_full()) {
        if (_txqueue.full()) {
            return DrvStatus::overflow;
        }
        _txqueue.push(frame);
        return DrvStatus::busy;
    }
    
    uint32_t mailboxid = read_bit<uint32_t>(_reg->TSR, CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
    if (mailboxid > 2) {
        return DrvStatus::error;
    }

    // set up id
    if (frame.id <= 0x7FF) {
        write_reg(_reg->sTxMailBox[mailboxid].TIR, (frame.id << CAN_TI0R_STID_Pos));
    } else if (frame.id <=0x1FFFFFFF) {
        write_reg(_reg->sTxMailBox[mailboxid].TIR, (frame.id << CAN_TI0R_EXID_Pos));
        set_bit<uint32_t>(_reg->sTxMailBox[mailboxid].TIR, CAN_TI0R_IDE);
    } else {
        return DrvStatus::invalid_argument;
    }

    // set up dlc
    write_reg(_reg->sTxMailBox[mailboxid].TDTR, static_cast<uint32_t>(frame.len));

    // set up data field
    write_reg(_reg->sTxMailBox[mailboxid].TDLR,
        (uint32_t(frame.payload[0]) << CAN_TDL0R_DATA0_Pos) |
        (uint32_t(frame.payload[1]) << CAN_TDL0R_DATA1_Pos) |
        (uint32_t(frame.payload[2]) << CAN_TDL0R_DATA2_Pos) |
        (uint32_t(frame.payload[3]) << CAN_TDL0R_DATA3_Pos));
    write_reg(_reg->sTxMailBox[mailboxid].TDHR,
        (uint32_t(frame.payload[4]) << CAN_TDH0R_DATA4_Pos) |
        (uint32_t(frame.payload[5]) << CAN_TDH0R_DATA5_Pos) |
        (uint32_t(frame.payload[6]) << CAN_TDH0R_DATA6_Pos) |
        (uint32_t(frame.payload[7]) << CAN_TDH0R_DATA7_Pos));
    
    // request transmission
    set_bit<uint32_t>(_reg->sTxMailBox[mailboxid].TIR, CAN_TI0R_TXRQ);

    return DrvStatus::ok;
}


std::optional<RxMessageAttribute> Module::recv(can_frame& frame, RxFifo fifo) const {
    if (rxfifo_level(fifo) == 0) {
        return {};
    }

    auto fifo_idx = std::to_underlying(fifo);

    // get id, len, filter
    if (bit_is_clear<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RIR, CAN_RI0R_IDE)) {
        frame.id = read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RIR, CAN_RI0R_STID) >> CAN_TI0R_STID_Pos;
    } else {
        frame.id = read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RIR, (CAN_RI0R_EXID | CAN_RI0R_STID)) >> CAN_RI0R_EXID_Pos;
    }

    frame.len = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDTR, CAN_RDT0R_DLC) >> CAN_RDT0R_DLC_Pos);

    RxMessageAttribute attr{};
    attr.filter_idx = read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDTR, CAN_RDT0R_FMI) >> CAN_RDT0R_FMI_Pos;
    attr.fifo = fifo;

    // get data
    frame.payload[0] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDLR, CAN_RDL0R_DATA0) >> CAN_RDL0R_DATA0_Pos);
    frame.payload[1] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDLR, CAN_RDL0R_DATA1) >> CAN_RDL0R_DATA1_Pos);
    frame.payload[2] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDLR, CAN_RDL0R_DATA2) >> CAN_RDL0R_DATA2_Pos);
    frame.payload[3] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDLR, CAN_RDL0R_DATA3) >> CAN_RDL0R_DATA3_Pos);
    frame.payload[4] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDHR, CAN_RDH0R_DATA4) >> CAN_RDH0R_DATA4_Pos);
    frame.payload[5] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDHR, CAN_RDH0R_DATA5) >> CAN_RDH0R_DATA5_Pos);
    frame.payload[6] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDHR, CAN_RDH0R_DATA6) >> CAN_RDH0R_DATA6_Pos);
    frame.payload[7] = uint8_t(read_bit<uint32_t>(_reg->sFIFOMailBox[fifo_idx].RDHR, CAN_RDH0R_DATA7) >> CAN_RDH0R_DATA7_Pos);

    // release fifo
    switch (fifo) {
    case RxFifo::fifo0:
        set_bit<uint32_t>(_reg->RF0R, CAN_RF0R_RFOM0);
        break;
    case RxFifo::fifo1:
        set_bit<uint32_t>(_reg->RF1R, CAN_RF1R_RFOM1);
        break;
    }    

    return {attr};
}


void Module::initialize_interrupts(uint32_t interrupt_list) {
    set_bit(_reg->IER, interrupt_list);
}


void Module::set_interrupt_priority(IrqPriority rx0_priority, IrqPriority rx1_priority, IrqPriority tx_priority) {
    set_irq_priority(impl::can_rx0_irqn[std::to_underlying(_peripheral)], rx0_priority);
    set_irq_priority(impl::can_rx1_irqn[std::to_underlying(_peripheral)], rx1_priority);
    set_irq_priority(impl::can_tx_irqn[std::to_underlying(_peripheral)], tx_priority);

}

void Module::enable_interrupts() {
    enable_irq(impl::can_rx0_irqn[std::to_underlying(_peripheral)]);
    enable_irq(impl::can_rx1_irqn[std::to_underlying(_peripheral)]);
    enable_irq(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
}

void Module::disable_interrupts() {
    disable_irq(impl::can_rx0_irqn[std::to_underlying(_peripheral)]);
    disable_irq(impl::can_rx1_irqn[std::to_underlying(_peripheral)]);
    disable_irq(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
}


void Module::_enable_clk(Peripheral peripheral) {
    auto can_idx = std::to_underlying(peripheral);
    if (_clk_enabled[can_idx]) {
        return;
    }
        
    impl::can_clk_enable_funcs[can_idx]();
    _clk_enabled[can_idx] = true;
}


} // namespace can
} // namespace mcu


extern "C" void CAN2_RX0_IRQHandler() {
    using namespace mcu::can;
    HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
}


extern "C" void CAN1_RX1_IRQHandler() {
    using namespace mcu::can;
    HAL_CAN_IRQHandler(Module::instance(Peripheral::can1)->handle());
}


extern "C" void CAN2_RX1_IRQHandler() {
    using namespace mcu::can;
    HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
}


extern "C" void CAN2_TX_IRQHandler() {
    using namespace mcu::can;
    HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
}


#endif
#endif
