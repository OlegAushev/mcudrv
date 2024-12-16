#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <apm32f4xx_dma.h>
#include <apm32f4xx_rcm.h>
#include <emblib/core.h>
#include <array>
#include <utility>


namespace mcu {
namespace dma {


enum class StreamId : unsigned int {
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


constexpr size_t stream_count = 16;


struct Config {
    StreamId stream_id;
    DMA_Config_T hal_config;
};


namespace impl {


inline std::array<void(*)(void), 2> dma_clk_enable_funcs = {
    [](){ RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA1); },
    [](){ RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA2); },
};


inline constexpr std::array<IRQn_Type, stream_count> dma_irqn = {
    DMA1_STR0_IRQn, DMA1_STR1_IRQn, DMA1_STR2_IRQn, DMA1_STR3_IRQn,
    DMA1_STR4_IRQn, DMA1_STR5_IRQn, DMA1_STR6_IRQn, DMA1_STR7_IRQn,
    DMA2_STR0_IRQn, DMA2_STR1_IRQn, DMA2_STR2_IRQn, DMA2_STR3_IRQn,
    DMA2_STR4_IRQn, DMA2_STR5_IRQn, DMA2_STR6_IRQn, DMA2_STR7_IRQn
};


inline const std::array<DMA_Stream_T*, stream_count> dma_stream_instances = {
    DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
    DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
    DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
    DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
};


} // namespace impl


class Stream : public emb::singleton_array<Stream, stream_count>, public emb::noncopyable {
private:
    const StreamId _stream_id;
    DMA_Stream_T* _stream_reg;
    //impl::DMA_Base_Registers* _base_reg;

    static inline std::array<bool, 2> _clk_enabled{false, false};
public:
    Stream(Config config);

    DMA_Stream_T* stream_reg() { return _stream_reg; }
    static Stream* instance(StreamId stream_id) {
        return emb::singleton_array<Stream, stream_count>::instance(std::to_underlying(stream_id));
    }

    void init_interrupts(uint32_t interrupt_bitset, mcu::IrqPriority priority);

    void enable_interrupts() {
        enable_irq(impl::dma_irqn[std::to_underlying(_stream_id)]);
    }

    void disable_interrupts() {
        disable_irq(impl::dma_irqn[std::to_underlying(_stream_id)]);
    }

    void enable() {
        _stream_reg->SCFG_B.EN = 1;
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
        _stream.stream_reg()->NDATA = uint32_t(_data.size());
        _stream.stream_reg()->M0ADDR = uint32_t(_data.data());
    }

    const T* data() const { return _data.data(); }
    constexpr uint32_t size() const { return _data.size(); }
    T& operator[](size_t pos) { return _data[pos]; }
    const T& operator[](size_t pos) const { return _data[pos]; }
};


template <typename T, size_t Size>
class MemoryDoubleBuffer {
private:
    Stream& _stream;
public:
    MemoryBuffer<T, Size> buf0;
    MemoryBuffer<T, Size> buf1;
    MemoryDoubleBuffer(Stream& stream) : _stream(stream), buf0(stream), buf1(stream) {
        _stream.stream_reg()->SCFG_B.DBM = 1;
        _stream.stream_reg()->NDATA = uint32_t(Size);
        _stream.stream_reg()->M0ADDR = uint32_t(buf0.data());
        _stream.stream_reg()->M1ADDR = uint32_t(buf1.data());
    }
};


} // namespace dma
} // namespace mcu


#endif
#endif
