#pragma once


#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include <array>
#include <utility>


namespace mcu {

namespace dma {

enum class StreamId {
    dma1_stream0,
    dma1_stream1,
    dma1_stream2,
    dma1_stream3,
    dma1_stream4,
    dma1_stream5,
    dma1_stream6,
    dma1_stream7,
    dma2_stream0,
    dma2_stream1,
    dma2_stream2,
    dma2_stream3,
    dma2_stream4,
    dma2_stream5,
    dma2_stream6,
    dma2_stream7
};


constexpr int stream_count = 16;


struct Config {
    StreamId stream_id;
    DMA_InitTypeDef hal_config;
};


namespace impl {


inline constexpr std::array<IRQn_Type, stream_count> dma_irq_numbers = {	
    DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
    DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
};


inline constexpr std::array<DMA_Stream_TypeDef*, stream_count> dma_stream_instances = {
    DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
    DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
    DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
    DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
};


} // namespace impl


class Stream : public emb::interrupt_invoker_array<Stream, stream_count>, public emb::noncopyable {
private:
    const StreamId _stream_id;
    DMA_HandleTypeDef _handle = {};
    DMA_Stream_TypeDef* _stream_reg;
    DMA_HandleTypeDef* _peripheral_handle = nullptr;

    static inline std::array<bool, 2> _clk_enabled = {false, false};
public:
    Stream(const Config& config);

    DMA_HandleTypeDef* handle() { return &_handle; }
    DMA_Stream_TypeDef* stream_reg() { return _stream_reg; }
    DMA_HandleTypeDef* peripheral_handle() { return _peripheral_handle; }
    static Stream* instance(StreamId stream_id) {
        return emb::interrupt_invoker_array<Stream, stream_count>::instance(std::to_underlying(stream_id));
    }

    void init_interrupts(DMA_HandleTypeDef* peripheral_handle, mcu::InterruptPriority priority) {
        _peripheral_handle = peripheral_handle;
        HAL_NVIC_SetPriority(impl::dma_irq_numbers[std::to_underlying(_stream_id)], priority.get(), 0);
    }

    void enable_interrupts() {
        HAL_NVIC_EnableIRQ(impl::dma_irq_numbers[std::to_underlying(_stream_id)]);
    }

    void disable_interrupts() {
        HAL_NVIC_DisableIRQ(impl::dma_irq_numbers[std::to_underlying(_stream_id)]);
    }
protected:
    static void _enable_clk(StreamId stream_id);
};


} // namespace dma

} // namespace mcu


#endif
