#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/dma/dma.h>


namespace mcu {


namespace dma {


Stream::Stream(Config config)
        : emb::interrupt_invoker_array<Stream, stream_count>(this, std::to_underlying(config.stream_id))
        , _stream_id(config.stream_id)
{
    _enable_clk(_stream_id);
    _stream_reg = impl::dma_stream_instances[std::to_underlying(_stream_id)];
    DMA_Config(_stream_reg, &config.hal_config);
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


void Stream::init_interrupts(uint32_t interrupt_bitset, mcu::IrqPriority priority) {
    if ((interrupt_bitset & DMA_INT_FEIFLG) == DMA_INT_FEIFLG) {
        _stream_reg->FCTRL_B.FEIEN = 1;
    }

    if (interrupt_bitset != DMA_INT_FEIFLG) {
        set_bit(_stream_reg->SCFG, interrupt_bitset & 0x1E);
    }

    set_irq_priority(impl::dma_irqn[std::to_underlying(_stream_id)], priority);
}


} // namespace dma


} // namespace mcu


#endif
#endif
