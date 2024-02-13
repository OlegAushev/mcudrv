#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/stm32_h7_base.h>
#include <mcudrv/stm32/h7/system/system.h>
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
    DMA_InitTypeDef hal_config;
};


namespace impl {


inline std::array<void(*)(void), 2> dma_clk_enable_funcs = {
    [](){ __HAL_RCC_DMA1_CLK_ENABLE(); },
    [](){ __HAL_RCC_DMA2_CLK_ENABLE(); },
};


inline constexpr std::array<IRQn_Type, stream_count> dma_irqn = {	
    DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
    DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
};


inline const std::array<DMA_Stream_TypeDef*, stream_count> dma_stream_instances = {
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
    DMA_HandleTypeDef _handle{};
    DMA_Stream_TypeDef* _stream_reg;
    impl::DMA_Base_Registers* _base_reg;

    static inline std::array<bool, 2> _clk_enabled{false, false};
public:
    Stream(const Config& config);

    DMA_HandleTypeDef* handle() { return &_handle; }
    DMA_Stream_TypeDef* stream_reg() { return _stream_reg; }
    static Stream* instance(StreamId stream_id) {
        return emb::interrupt_invoker_array<Stream, stream_count>::instance(std::to_underlying(stream_id));
    }

    void init_interrupts(uint32_t interrupt_list, mcu::IrqPriority priority);

    void enable_interrupts() {
        enable_irq(impl::dma_irqn[std::to_underlying(_stream_id)]);
    }

    void disable_interrupts() {
        disable_irq(impl::dma_irqn[std::to_underlying(_stream_id)]);
    }

    void enable() {
        set_bit<uint32_t>(_stream_reg->CR, DMA_SxCR_EN);
    }
protected:
    static void _enable_clk(StreamId stream_id);



//     StreamController(Stream stream, const Config& config)
//             : emb::interrupt_invoker_array<StreamController, stream_count>(this, std::to_underlying(stream))
//             , _stream(stream) {
//         enable_clk();
//         _handle.Instance = impl::dma_stream_instances[std::to_underlying(_stream)];
//         _handle.Init = config.hal_init;
//         if (HAL_DMA_DeInit(&_handle) != HAL_OK) {
//             fatal_error("DMA stream deinitialization failed");
//         }
//         if (HAL_DMA_Init(&_handle) != HAL_OK) {
//             fatal_error("DMA stream initialization failed");
//         }
//     }

//     DMA_HandleTypeDef* handle() { return &_handle; }
//     DMA_HandleTypeDef* peripheral_handle() { return _peripheral_handle; }
//     static StreamController* instance(Stream stream) {
//         return emb::interrupt_invoker_array<StreamController, stream_count>::instance(std::to_underlying(stream));
//     }


//     void init_interrupts(DMA_HandleTypeDef* peripheral_handle, mcu::IrqPriority priority) {
//         _peripheral_handle = peripheral_handle;
//         HAL_NVIC_SetPriority(impl::irq_numbers[std::to_underlying(_stream)], priority.get(), 0);
//     }

//     void enable_interrupts() {
//         HAL_NVIC_EnableIRQ(impl::irq_numbers[std::to_underlying(_stream)]);
//     }

//     void disable_interrupts() {
//         HAL_NVIC_DisableIRQ(impl::irq_numbers[std::to_underlying(_stream)]);
//     }
// protected:
//     void enable_clk() {
//         switch (_stream) {
//         case Stream::dma1_stream0:
//         case Stream::dma1_stream1:
//         case Stream::dma1_stream2:
//         case Stream::dma1_stream3:
//         case Stream::dma1_stream4:
//         case Stream::dma1_stream5:
//         case Stream::dma1_stream6:
//         case Stream::dma1_stream7:
//             if (_clk_enabled[0]) return;
//             __HAL_RCC_DMA1_CLK_ENABLE();
//             _clk_enabled[0] = true;
//             break;
//         case Stream::dma2_stream0:
//         case Stream::dma2_stream1:
//         case Stream::dma2_stream2:
//         case Stream::dma2_stream3:
//         case Stream::dma2_stream4:
//         case Stream::dma2_stream5:
//         case Stream::dma2_stream6:
//         case Stream::dma2_stream7:
//             if (_clk_enabled[1]) return;
//             __HAL_RCC_DMA2_CLK_ENABLE();
//             _clk_enabled[1] = true;
//             break;
//         default:
//             fatal_error("invalid DMA stream");
//             break;
//         }
//     }
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

    void invalidate_dcache(size_t offset_, size_t size_) {
        SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(&_data[offset_]), int32_t(size_ * sizeof(T)));
    }
};


template <typename T, size_t Size>
class MemoryDoubleBuffer {
private:
    Stream& _stream;
public:
    MemoryBuffer<T, Size> buf0;
    MemoryBuffer<T, Size> buf1;
    MemoryDoubleBuffer(Stream& stream) : _stream(stream), buf0(stream), buf1(stream) {
        set_bit<uint32_t>(_stream.stream_reg()->CR, DMA_SxCR_DBM);
        write_reg(_stream.stream_reg()->NDTR, uint32_t(Size));
        write_reg(_stream.stream_reg()->M0AR, uint32_t(buf0.data()));
        write_reg(_stream.stream_reg()->M1AR, uint32_t(buf1.data()));
    }
};


// template <typename T, uint32_t Size>
// struct Buffer {
// private:
//     T _data[Size] __attribute__((aligned(32)));
// public:
//     T* data() { return _data; }
//     constexpr const T* data() const { return _data; }

//     constexpr uint32_t size() const { return Size; }

//     T& operator[](size_t pos) { return _data[pos]; }
//     constexpr T& operator[](size_t pos) const { return _data[pos]; }

//     void invalidate_dcache(size_t offset_, size_t size_) {
//         SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(&_data[offset_]), int32_t(size_ * sizeof(T)));
//     }
// };


} // namespace dma


} // namespace mcu


#endif
#endif
