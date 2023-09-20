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


inline std::array<void(*)(void), 2> dma_clk_enable_funcs = {
    [](){ __HAL_RCC_DMA1_CLK_ENABLE(); },
    [](){ __HAL_RCC_DMA2_CLK_ENABLE(); },
};


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


// already defined in stm32f4xx_hal_dma.c, but is private
struct DMA_Base_Registers
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
};


} // namespace impl


class Stream : public emb::interrupt_invoker_array<Stream, stream_count>, public emb::noncopyable {
private:
    const StreamId _stream_id;
    DMA_HandleTypeDef _handle = {};
    DMA_Stream_TypeDef* _stream_reg;
    impl::DMA_Base_Registers* _base_reg;

    static inline std::array<bool, 2> _clk_enabled = {false, false};
public:
    Stream(const Config& config);

    DMA_HandleTypeDef* handle() { return &_handle; }
    DMA_Stream_TypeDef* stream_reg() { return _stream_reg; }
    static Stream* instance(StreamId stream_id) {
        return emb::interrupt_invoker_array<Stream, stream_count>::instance(std::to_underlying(stream_id));
    }

    void init_interrupts(uint32_t interrupt_list,mcu::InterruptPriority priority);

    void enable_interrupts() {
        enable_interrupt(impl::dma_irq_numbers[std::to_underlying(_stream_id)]);
    }

    void disable_interrupts() {
        disable_interrupt(impl::dma_irq_numbers[std::to_underlying(_stream_id)]);
    }

    void enable() {
        set_bit(_stream_reg->CR, DMA_SxCR_EN);
    }
protected:
    static void _enable_clk(StreamId stream_id);
};


template <typename T, size_t Size>
class MemoryBuffer {
private:
    std::array<T, Size> _data __attribute__((aligned(32)));
    Stream& _stream;
public:
    MemoryBuffer(Stream& stream) : _stream(stream) {
        write_reg(_stream.stream_reg()->NDTR, uint32_t(_data.size()));
        write_reg(_stream.stream_reg()->M0AR, uint32_t(_data.data()));
    }

    constexpr const T* data() const { return _data.data(); }
    constexpr uint32_t size() const { return _data.size(); }
    T& operator[](size_t pos) { return _data[pos]; }
    constexpr T& operator[](size_t pos) const { return _data[pos]; }
};


template <typename T, size_t Size>
class MemoryDoubleBuffer {
private:
    Stream& _stream;
public:
    MemoryBuffer<T, Size> buf0;
    MemoryBuffer<T, Size> buf1;
    MemoryDoubleBuffer(Stream& stream) : _stream(stream), buf0(stream), buf1(stream) {
        set_bit(_stream.stream_reg()->CR, DMA_SxCR_DBM);
        write_reg(_stream.stream_reg()->NDTR, uint32_t(Size));
        write_reg(_stream.stream_reg()->M0AR, uint32_t(buf0.data()));
        write_reg(_stream.stream_reg()->M1AR, uint32_t(buf1.data()));
    }
};


} // namespace dma

} // namespace mcu


#endif
