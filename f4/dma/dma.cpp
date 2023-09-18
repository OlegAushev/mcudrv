#ifdef STM32F4xx

#include <mculib_stm32/f4/dma/dma.h>


namespace mcu {

namespace dma {


Stream::Stream(const Config& config)
        : emb::interrupt_invoker_array<Stream, stream_count>(this, std::to_underlying(config.stream_id))
        , _stream_id(config.stream_id) {
    _enable_clk(_stream_id);
    _handle.Instance = impl::dma_stream_instances[std::to_underlying(_stream_id)];
    _handle.Init = config.hal_config;

    _stream_reg = _handle.Instance;

    if (HAL_DMA_DeInit(&_handle) != HAL_OK) {
        fatal_error("DMA stream deinitialization failed");
    }
    if (HAL_DMA_Init(&_handle) != HAL_OK) {
        fatal_error("DMA stream initialization failed");
    }
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
        if (_clk_enabled[0]) return;
        __HAL_RCC_DMA1_CLK_ENABLE();
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
        if (_clk_enabled[1]) return;
        __HAL_RCC_DMA2_CLK_ENABLE();
        _clk_enabled[1] = true;
        break;
    }
}


} // namespace dma

} // namespace mcu


#endif
