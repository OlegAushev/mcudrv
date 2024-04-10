#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/can/can.h>
#include <mcudrv/apm32/f4/chrono/chrono.h>
#include <emblib/chrono.h>


namespace mcu {
namespace can {


Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, Config config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _rx_pin.initialize({.port = rx_pin_config.port,
                  .pin = {.pin = rx_pin_config.pin,
                          .mode = GPIO_MODE_AF,
                          .speed = GPIO_SPEED_50MHz,
                          .otype = GPIO_OTYPE_PP,
                          .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = rx_pin_config.altfunc,
                  .actstate{}});

    _tx_pin.initialize({.port = tx_pin_config.port,
                  .pin = {.pin = tx_pin_config.pin,
                          .mode = GPIO_MODE_AF,
                          .speed = GPIO_SPEED_50MHz,
                          .otype = GPIO_OTYPE_PP,
                          .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = tx_pin_config.altfunc,
                  .actstate{}});

    _enable_clk(peripheral);
    _reg = impl::can_instances[static_cast<size_t>(_peripheral)];

    CAN_Config(_reg, &config.hal_config);
}


RxMessageAttribute Module::register_rxmessage(CAN_FilterConfig_T& filter) {
    RxMessageAttribute attr = {};
    
    if (_filter_count >= max_fitler_count) {
        fatal_error("too many CAN Rx filters");
    }

    if (_peripheral == Peripheral::can1) {
        filter.filterNumber = _filter_count++;
    } else {
        filter.filterNumber = 14 + _filter_count++;
    }
    filter.filterActivation = ENABLE;
    //filter.SlaveStartFilterBank = 14;

    attr.filter_idx = filter.filterNumber;
    attr.fifo = RxFifo(filter.filterFIFO);

    CAN_ConfigFilter(&filter);

    return attr;
}


void Module::start() {
    _reg->MCTRL_B.INITREQ = 0;
    emb::chrono::timeout start_timeout(std::chrono::milliseconds(2));
    while (_reg->MSTS_B.INITFLG == 1) {
        if (start_timeout.expired()) {
             fatal_error("CAN module start failed");
        }
    }
}


void Module::stop() {
    _reg->MCTRL_B.INITREQ = 1;
    emb::chrono::timeout stop_timeout(std::chrono::milliseconds(2));
    while (_reg->MSTS_B.INITFLG == 0) {
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
    
    uint32_t mailboxid = _reg->TXSTS_B.EMNUM;
    if (mailboxid > 2) {
        return DrvStatus::error;
    }

    // set up id
    if (frame.id <= 0x7FF) {
        write_reg(_reg->sTxMailBox[mailboxid].TXMID, (frame.id << 21));
    } else if (frame.id <=0x1FFFFFFF) {
        write_reg(_reg->sTxMailBox[mailboxid].TXMID, (frame.id << 3));
        _reg->sTxMailBox[mailboxid].TXMID_B.IDTYPESEL = 1;
    } else {
        return DrvStatus::invalid_argument;
    }

    // set up dlc
    _reg->sTxMailBox[mailboxid].TXDLEN_B.DLCODE = frame.len;

    // set up data field
    write_reg(_reg->sTxMailBox[mailboxid].TXMDL,
        (uint32_t(frame.payload[0]) << 0) |
        (uint32_t(frame.payload[1]) << 8) |
        (uint32_t(frame.payload[2]) << 16) |
        (uint32_t(frame.payload[3]) << 24));
    write_reg(_reg->sTxMailBox[mailboxid].TXMDH,
        (uint32_t(frame.payload[4]) << 0) |
        (uint32_t(frame.payload[5]) << 8) |
        (uint32_t(frame.payload[6]) << 16) |
        (uint32_t(frame.payload[7]) << 24));
    
    // request transmission
    _reg->sTxMailBox[mailboxid].TXMID_B.TXMREQ = 1;

    return DrvStatus::ok;
}


std::optional<RxMessageAttribute> Module::recv(can_frame& frame, RxFifo fifo) const {
    if (rxfifo_level(fifo) == 0) {
        return {};
    }

    auto fifo_idx = std::to_underlying(fifo);

    // get id, len, filter
    if (_reg->sRxMailBox[fifo_idx].RXMID_B.IDTYPESEL == 0) {
        frame.id = _reg->sRxMailBox[fifo_idx].RXMID >> 21;
    } else {
        frame.id = _reg->sRxMailBox[fifo_idx].RXMID >> 3;
    }

    frame.len = uint8_t(_reg->sRxMailBox[fifo_idx].RXDLEN_B.DLCODE);

    RxMessageAttribute attr{};
    attr.filter_idx = _reg->sRxMailBox[fifo_idx].RXDLEN_B.FMIDX;
    attr.fifo = fifo;

    // get data
    frame.payload[0] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE0);
    frame.payload[1] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE1);
    frame.payload[2] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE2);
    frame.payload[3] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE3);
    frame.payload[4] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE4);
    frame.payload[5] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE5);
    frame.payload[6] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE6);
    frame.payload[7] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE7);

    // release fifo
    switch (fifo) {
    case RxFifo::fifo0:
        _reg->RXF0_B.RFOM0 = 1;
        break;
    case RxFifo::fifo1:
        _reg->RXF1_B.RFOM1 = 1;;
        break;
    }    

    return {attr};
}


void Module::initialize_interrupts(uint32_t interrupt_list) {
    set_bit(_reg->INTEN, interrupt_list);
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


// extern "C" void CAN2_RX0_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }


// extern "C" void CAN1_RX1_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can1)->handle());
// }


// extern "C" void CAN2_RX1_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }


// extern "C" void CAN2_TX_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }


#endif
#endif
