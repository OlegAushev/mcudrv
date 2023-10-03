#ifdef STM32F4xx

#include <mculib_stm32/f4/dma/dma.h>


namespace mcu {
namespace dma {


Stream::Stream(const Config& config)
        : emb::interrupt_invoker_array<Stream, stream_count>(this, std::to_underlying(config.stream_id))
        , _stream_id(config.stream_id) {
    _enable_clk(_stream_id);
    _stream_reg = impl::dma_stream_instances[std::to_underlying(_stream_id)];
    _handle.Instance = _stream_reg;
    _handle.Init = config.hal_config;

    if (HAL_DMA_DeInit(&_handle) != HAL_OK) {
        fatal_error("DMA stream deinitialization failed");
    }
    if (HAL_DMA_Init(&_handle) != HAL_OK) {
        fatal_error("DMA stream initialization failed");
    }

    _base_reg = reinterpret_cast<impl::DMA_Base_Registers*>(_handle.StreamBaseAddress);
}


void Stream::_enable_clk(StreamId stream_id) {
    switch (stream_id) {
    case StreamId::dma1_stream0:
    case StreamId::dma1_stream1:
    case StreamId::dma1_stream2:
    case StreamId::dma1_stream3:
    case StreamId::dma1_stream4:
    case StreamId::dma1_stream5:
    case StreamId::dma1_stream6:
    case StreamId::dma1_stream7:
        if (_clk_enabled[0]) {
            return;
        }
        impl::dma_clk_enable_funcs[0]();
        _clk_enabled[0] = true;
        break;
    case StreamId::dma2_stream0:
    case StreamId::dma2_stream1:
    case StreamId::dma2_stream2:
    case StreamId::dma2_stream3:
    case StreamId::dma2_stream4:
    case StreamId::dma2_stream5:
    case StreamId::dma2_stream6:
    case StreamId::dma2_stream7:
        if (_clk_enabled[1]) {
            return;
        }
        impl::dma_clk_enable_funcs[1]();
        _clk_enabled[1] = true;
        break;
    }
}


void Stream::init_interrupts(uint32_t interrupt_list,mcu::IrqPriority priority) {
    write_reg(_base_reg->IFCR, 0x3FUL << _handle.StreamIndex); // clear all interrupt flags
    set_bit(_stream_reg->CR, interrupt_list);
    set_irq_priority(impl::dma_irqn[std::to_underlying(_stream_id)], priority);
}


} // namespace dma
} // namespace mcu


#endif
