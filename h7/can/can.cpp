#include <mculib_stm32/h7/can/can.h>


namespace mcu {

namespace can {

Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
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

    enable_clk();
    _handle.Instance = impl::can_instances[static_cast<size_t>(_peripheral)];
    _handle.Init = config.hal_init;
    if (HAL_FDCAN_Init(&_handle) != HAL_OK) {
        fatal_error("CAN module initialization failed");
    }

    // Configure global filter to reject all non-matching frames
    HAL_FDCAN_ConfigGlobalFilter(&_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // Default interrupt config
    init_interrupts(FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_BUFFER_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
	init_interrupts(FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1);
}


MessageAttribute Module::register_message(FDCAN_FilterTypeDef& filter) {
    MessageAttribute attr;

    if ((filter.IdType == FDCAN_STANDARD_ID) && (_stdfilter_count >= _handle.Init.StdFiltersNbr)) {
        fatal_error("too many CAN Rx std filters");
    }

    if ((filter.IdType == FDCAN_EXTENDED_ID) && (_extfilter_count >= _handle.Init.ExtFiltersNbr)) {
        fatal_error("too many CAN Rx ext filters");
    }

    if (filter.IdType == FDCAN_STANDARD_ID) {
        filter.FilterIndex = _stdfilter_count++;
    } else {
        filter.FilterIndex = _extfilter_count++;
    }

    attr.filter_index = filter.FilterIndex;

    if (filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0 || filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0_HP) {
        attr.location = FDCAN_RX_FIFO0;
    } else if (filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO1 || filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO1_HP) {
        attr.location = FDCAN_RX_FIFO1;
    } else if (filter.FilterConfig == FDCAN_FILTER_TO_RXBUFFER) {
        if (_rxbuffer_count >= _handle.Init.RxBuffersNbr) {
            fatal_error("too many CAN Rx buffers");
        } else {
            filter.RxBufferIndex = _rxbuffer_count;
            attr.location = _rxbuffer_count++;
        }
    } else {
        fatal_error("CAN module Rx filter configuration failed");
    }

    if (HAL_FDCAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        fatal_error("CAN module Rx filter configuration failed");
    }

    return attr;
}


void Module::init_interrupts(uint32_t interrupt_list, uint32_t interrupt_line)
{
    if (HAL_FDCAN_ConfigInterruptLines(&_handle, interrupt_list, interrupt_line) != HAL_OK) {
        fatal_error("CAN interrupt configuration failed");
    }

    if (HAL_FDCAN_ActivateNotification(&_handle, interrupt_list, 0) != HAL_OK) {
        fatal_error("CAN interrupt configuration failed");
    }
}


void Module::set_fifo_watermark(uint32_t fifo, uint32_t watermark)
{
    if (HAL_FDCAN_ConfigFifoWatermark(&_handle, fifo, watermark) != HAL_OK) {
        emb::fatal_error("CAN interrupt configuration failed");
    }
}

} // namespace can

} // namespace mcu

extern "C" void FDCAN1_IT0_IRQHandler(void) {
    using namespace mcu::can;
    HAL_FDCAN_IRQHandler(Module::instance(Peripheral::fdcan1)->handle());
}


extern "C" void FDCAN2_IT0_IRQHandler(void) {
    using namespace mcu::can;
    HAL_FDCAN_IRQHandler(Module::instance(Peripheral::fdcan2)->handle());
}


extern "C" void FDCAN1_IT1_IRQHandler(void) {
    using namespace mcu::can;
    HAL_FDCAN_IRQHandler(Module::instance(Peripheral::fdcan1)->handle());
}


extern "C" void FDCAN2_IT1_IRQHandler(void) {
    using namespace mcu::can;
    HAL_FDCAN_IRQHandler(Module::instance(Peripheral::fdcan2)->handle());
}


/*extern "C" void FDCAN_CAL_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan);
}*/


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t interrupt_flags) {
    using namespace mcu::can;
    if ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        do {
            FDCAN_RxHeaderTypeDef header;
            can_frame frame;
            HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame.payload.data());
            frame.id = header.Identifier;
            frame.len = header.DataLength >> 16;

            MessageAttribute attr;
            attr.location = FDCAN_RX_FIFO0;
            attr.filter_index = header.FilterIndex;

            auto module = Module::instance(impl::to_peripheral(handle->Instance));
            module->_on_fifo0_frame_received(*module, attr, frame);
        } while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0); 
    }
}


void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *handle) {
    using namespace mcu::can;
    Module::instance(impl::to_peripheral(handle->Instance))->_on_buffer_frame_received();
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* handle, uint32_t interrupt_flags) {
    using namespace mcu::can;
    if ((interrupt_flags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        do {
            FDCAN_RxHeaderTypeDef header;
            can_frame frame;
            HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO1, &header, frame.payload.data());
            frame.id = header.Identifier;
            frame.len = header.DataLength >> 16;
            
            MessageAttribute attr;
            attr.location = FDCAN_RX_FIFO1;
            attr.filter_index = header.FilterIndex;
            
            auto module = Module::instance(impl::to_peripheral(handle->Instance));
            module->_on_fifo1_frame_received(*module, attr, frame);
        } while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO1) > 0); 
    }
}

