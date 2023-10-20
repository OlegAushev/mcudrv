#ifdef STM32F4xx

#include <mculib_stm32/f4/can/can.h>


namespace mcu {

namespace can {

Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _rx_pin.init({
    .port = rx_pin_config.port,
    .pin = {
        .Pin = rx_pin_config.pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = rx_pin_config.af_selection
    },
    .active_state = emb::gpio::ActiveState::high});

    _tx_pin.init({
        .port = tx_pin_config.port,
        .pin = {
            .Pin = tx_pin_config.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = tx_pin_config.af_selection
        },
        .active_state = emb::gpio::ActiveState::high});

    _enable_clk(peripheral);

    _handle.Instance = impl::can_instances[static_cast<size_t>(_peripheral)];
    _handle.Init = config.hal_init;

    if(HAL_CAN_Init(&_handle) != HAL_OK) {
        fatal_error("CAN module initialization failed");
    }

    // Default interrupt config
    // init_interrupts(CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}


MessageAttribute Module::register_message(CAN_FilterTypeDef& filter) {
    MessageAttribute attr = {};
    
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
    attr.filter_index = filter.FilterBank;
    attr.location = filter.FilterFIFOAssignment;

    if (HAL_CAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        fatal_error("CAN module Rx filter configuration failed");
    }

    return attr;
}


void Module::init_interrupts(uint32_t interrupt_list) {
    if (HAL_CAN_ActivateNotification(&_handle, interrupt_list) != HAL_OK) {
        fatal_error("CAN interrupt configuration failed");
    }
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


//------------------------------------------------------------------------------


// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle) {
//     using namespace mcu::can;
//     do {
//             CAN_RxHeaderTypeDef header;
//             can_frame frame;
//             HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, frame.payload.data());
//             frame.id = header.StdId;
//             frame.len = header.DLC;

//             MessageAttribute attr;
//             attr.location = CAN_RX_FIFO0;
//             attr.filter_index = header.FilterMatchIndex;

//             auto module = Module::instance(impl::to_peripheral(handle->Instance));
//             module->_on_fifo0_frame_received(*module, attr, frame);
//     } while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) > 0);
// }


// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* handle) {
//     using namespace mcu::can;
//     do {
//             CAN_RxHeaderTypeDef header;
//             can_frame frame;
//             HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO1, &header, frame.payload.data());
//             frame.id = header.StdId;
//             frame.len = header.DLC;

//             MessageAttribute attr;
//             attr.location = CAN_RX_FIFO1;
//             attr.filter_index = header.FilterMatchIndex;

//             auto module = Module::instance(impl::to_peripheral(handle->Instance));
//             module->_on_fifo0_frame_received(*module, attr, frame);
//     } while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO1) > 0);
// }


// void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* handle) {
//     using namespace mcu::can;
//     auto module = Module::instance(impl::to_peripheral(handle->Instance));
//     module->_on_txmailbox_free(*module);
// }


// void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* handle) {
//     using namespace mcu::can;
//     auto module = Module::instance(impl::to_peripheral(handle->Instance));
//     module->_on_txmailbox_free(*module);
// }


// void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* handle) {
//     using namespace mcu::can;
//     auto module = Module::instance(impl::to_peripheral(handle->Instance));
//     module->_on_txmailbox_free(*module);
// }


#endif
