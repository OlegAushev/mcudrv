#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/can/can.h>


namespace mcu {


namespace can {


Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
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

    enable_clk();
    _reg = impl::can_instances[std::to_underlying(_peripheral)];
    _handle.Instance = _reg;
    _handle.Init = config.hal_config;

    if (HAL_FDCAN_Init(&_handle) != HAL_OK) {
        fatal_error("CAN module initialization failed");
    }

    // Configure global filter to reject all non-matching frames
    HAL_FDCAN_ConfigGlobalFilter(&_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // Default interrupt config
    initialize_interrupts(FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_BUFFER_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
	initialize_interrupts(FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1);
}


RxMessageAttribute Module::register_message(FDCAN_FilterTypeDef& filter) {
    RxMessageAttribute attr;

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

    attr.id_type = filter.IdType;
    attr.filter_idx = filter.FilterIndex;

    if (filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0 || filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0_HP) {
        attr.storage = FDCAN_RX_FIFO0;
    } else if (filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO1 || filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO1_HP) {
        attr.storage = FDCAN_RX_FIFO1;
    } else if (filter.FilterConfig == FDCAN_FILTER_TO_RXBUFFER) {
        if (_rxbuffer_count >= _handle.Init.RxBuffersNbr) {
            fatal_error("too many CAN Rx buffers");
        } else {
            filter.RxBufferIndex = _rxbuffer_count;
            attr.storage = _rxbuffer_count++;
        }
    } else {
        fatal_error("CAN module Rx filter configuration failed");
    }

    if (HAL_FDCAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        fatal_error("CAN module Rx filter configuration failed");
    }

    return attr;
}


std::optional<RxMessageAttribute> Module::recv(can_frame& frame, RxFifo fifo) {
    if (rxfifo_level(fifo) == 0) {
        return {};
    }

    uint32_t* rx_addr;
    uint32_t index = 0;

    switch (fifo) {
    case RxFifo::fifo0:
        // check that the Rx FIFO 0 is full & overwrite mode is on
        if(((_reg->RXF0S & FDCAN_RXF0S_F0F) >> FDCAN_RXF0S_F0F_Pos) == 1) {
            if(((_reg->RXF0C & FDCAN_RXF0C_F0OM) >> FDCAN_RXF0C_F0OM_Pos) == FDCAN_RX_FIFO_OVERWRITE) {
                // when overwrite status is on discard first message in FIFO
                index = 1;
            }
        }
        // calculate Rx FIFO 0 element index
        index += ((_reg->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);
        // calculate Rx FIFO 0 element address
        rx_addr = reinterpret_cast<uint32_t*>(_handle.msgRam.RxFIFO0SA + (index * _handle.Init.RxFifo0ElmtSize * 4U));
        break;
    case RxFifo::fifo1:
        // check that the Rx FIFO 1 is full & overwrite mode is on
        if(((_reg->RXF1S & FDCAN_RXF1S_F1F) >> FDCAN_RXF1S_F1F_Pos) == 1) {
            if(((_reg->RXF1C & FDCAN_RXF1C_F1OM) >> FDCAN_RXF1C_F1OM_Pos) == FDCAN_RX_FIFO_OVERWRITE) {
                // when overwrite status is on discard first message in FIFO
                index = 1;
            }
        }
        // calculate Rx FIFO 1 element index
        index += ((_reg->RXF1S & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos);
        // calculate Rx FIFO 1 element address
        rx_addr = reinterpret_cast<uint32_t*>(_handle.msgRam.RxFIFO1SA + (index * _handle.Init.RxFifo1ElmtSize * 4U));
        break;
    }

    RxMessageAttribute attr{};

    attr.id_type = *rx_addr & 0x40000000U; //FDCAN_ELEMENT_MASK_XTD;
    if (attr.id_type == FDCAN_STANDARD_ID) {
        frame.id = (*rx_addr & 0x1FFC0000U) >> 18;
    } else {
        frame.id = *rx_addr & 0x1FFFFFFFU;
    }

    // increment RxAddress pointer to second word of Rx FIFO element
    ++rx_addr;

    frame.len = (*rx_addr & 0x000F0000U) >> 16;
    attr.filter_idx = (*rx_addr & 0x7F000000U) >> 24;
   
    // increment RxAddress pointer to payload of Rx FIFO element
    ++rx_addr;
    uint8_t* data = reinterpret_cast<uint8_t*>(rx_addr);
    for (size_t i = 0; i < frame.len; ++i) {
        frame.payload[i] = data[i];
    }

    switch(fifo) {
    case RxFifo::fifo0:
        attr.storage = FDCAN_RX_FIFO0;
        _reg->RXF0A = index;
        break;
    case RxFifo::fifo1:
        attr.storage = FDCAN_RX_FIFO1;
        _reg->RXF1A = index;
        break;
    }

    return {attr};
}


void Module::initialize_interrupts(uint32_t interrupt_list, uint32_t interrupt_line)
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
        fatal_error("CAN interrupt configuration failed");
    }
}


} // namespace can


} // namespace mcu


/*extern "C" void FDCAN_CAL_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan);
}*/


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t interrupt_flags) {
    using namespace mcu::can;
    if (mcu::bit_is_clear<uint32_t>(interrupt_flags, FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef header{};
        can_frame frame{};
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame.payload.data()) != HAL_OK) {
            return;
        }
        frame.id = header.Identifier;
        frame.len = uint8_t(header.DataLength >> 16);

        RxMessageAttribute attr{};
        attr.storage = FDCAN_RX_FIFO0;
        attr.id_type = header.IdType;
        attr.filter_idx = header.FilterIndex;

        auto module = Module::instance(impl::to_peripheral(handle->Instance));
        module->_on_fifo0_frame_received(*module, attr, frame);
    }
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* handle, uint32_t interrupt_flags) {
    using namespace mcu::can;
    if (mcu::bit_is_clear<uint32_t>(interrupt_flags, FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO1) > 0) {
        FDCAN_RxHeaderTypeDef header{};
        can_frame frame{};
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO1, &header, frame.payload.data()) != HAL_OK) {
            return;
        }
        frame.id = header.Identifier;
        frame.len = uint8_t(header.DataLength >> 16);
        
        RxMessageAttribute attr{};
        attr.storage = FDCAN_RX_FIFO1;
        attr.id_type = header.IdType;
        attr.filter_idx = header.FilterIndex;
        
        auto module = Module::instance(impl::to_peripheral(handle->Instance));
        module->_on_fifo1_frame_received(*module, attr, frame);
    } 
}


void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *handle) {
    using namespace mcu::can;
    Module::instance(impl::to_peripheral(handle->Instance))->_on_buffer_frame_received();
}


#endif
#endif
