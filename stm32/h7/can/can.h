#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/can.h>
#include <optional>
#include <utility>


namespace mcu {


namespace can {


inline constexpr bool strict_error_check = true;


enum class Peripheral : unsigned int {
    fdcan1,
    fdcan2
};


constexpr size_t peripheral_count = 2;


struct RxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct TxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct Config {
    FDCAN_InitTypeDef hal_config;
};


namespace impl {


inline constexpr std::array<IRQn_Type, 2> irq0_numbers = {	
    FDCAN1_IT0_IRQn,
    FDCAN2_IT0_IRQn,
};


inline constexpr std::array<IRQn_Type, 2> irq1_numbers = {	
    FDCAN1_IT1_IRQn,
    FDCAN2_IT1_IRQn,
};


inline const std::array<FDCAN_GlobalTypeDef*, peripheral_count> can_instances = {FDCAN1, FDCAN2};


inline Peripheral to_peripheral(const FDCAN_GlobalTypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(can_instances.begin(), std::find(can_instances.begin(), can_instances.end(), instance))
    );
}


inline constexpr std::array<uint32_t, 9> data_len_codes = {	
    FDCAN_DLC_BYTES_0,
    FDCAN_DLC_BYTES_1,
    FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,
    FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
};


} // namespace impl


enum class RxFifo {
    fifo0,
    fifo1
};


struct RxMessageAttribute {
    uint32_t storage{0xBAAAAAAD};
    uint32_t id_type{0xBAAAAAAD};
    uint32_t filter_idx{0xBAAAAAAD};
    bool operator==(const RxMessageAttribute&) const = default;
};


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
    friend void ::HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);
    friend void ::HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef*, uint32_t);
    friend void ::HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan);
private:
    const Peripheral _peripheral;
    FDCAN_HandleTypeDef _handle = {};
    FDCAN_GlobalTypeDef* _reg;
    mcu::gpio::AlternateIO _rx_pin;
    mcu::gpio::AlternateIO _tx_pin;

    static inline bool _clk_enabled = false;

    unsigned int _stdfilter_count = 0;
    unsigned int _extfilter_count = 0;
    unsigned int _rxbuffer_count = 0;

    uint64_t _tx_error_counter = 0;
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
    RxMessageAttribute register_message(FDCAN_FilterTypeDef& filter);
    
    Peripheral peripheral() const { return _peripheral; }
    FDCAN_HandleTypeDef* handle() { return &_handle; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void start() {
        if (HAL_FDCAN_Start(&_handle) != HAL_OK) {
            fatal_error("CAN module start failed");
        }
    }

    void stop() {
        if (HAL_FDCAN_Stop(&_handle) != HAL_OK) {
            fatal_error("CAN module stop failed");
        }
    }

    uint32_t rxfifo_level(RxFifo fifo) const {
        switch (fifo) {
        case RxFifo::fifo0:
            return read_bit<uint32_t>(_reg->RXF0S, FDCAN_RXF0S_F0FL);
        case RxFifo::fifo1:
            return read_bit<uint32_t>(_reg->RXF1S, FDCAN_RXF1S_F1FL);
        }
        return 0;
    }

    HalStatus send(can_frame& frame) {
        FDCAN_TxHeaderTypeDef header = {
            .Identifier = frame.id,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = impl::data_len_codes[frame.len],
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0
        };

        HalStatus status = HAL_FDCAN_AddMessageToTxFifoQ(&_handle, &header, frame.payload.data());
        if (status != HAL_OK) {
            ++_tx_error_counter;
        }
        return status;
    }

    HalStatus send(FDCAN_TxHeaderTypeDef& header, can_payload& payload) {
        HalStatus status = HAL_FDCAN_AddMessageToTxFifoQ(&_handle, &header, payload.data());
        if (status != HAL_OK) {
            ++_tx_error_counter;
        }
        return status;		
    }

    std::optional<RxMessageAttribute> recv(can_frame& frame, RxFifo fifo);

    /* INTERRUPTS */
private:
    void (*_on_fifo0_frame_received)(Module&, const RxMessageAttribute&, const can_frame&) = [](auto, auto, auto){ fatal_error("uninitialized callback"); };
    void (*_on_fifo1_frame_received)(Module&, const RxMessageAttribute&, const can_frame&) = [](auto, auto, auto){ fatal_error("uninitialized callback"); };
    void (*_on_buffer_frame_received)() = [](){ fatal_error("uninitialized callback"); };
public:
    void register_on_fifo0_frame_received_callback(void(*callback)(Module&, const RxMessageAttribute&, const can_frame&)) {
        _on_fifo0_frame_received = callback;
    }
    void register_on_fifo1_frame_received_callback(void(*callback)(Module&, const RxMessageAttribute&, const can_frame&)) {
        _on_fifo1_frame_received = callback;
    }
    void register_on_buffer_frame_received_callback(void(*callback)()) {
        _on_buffer_frame_received = callback;
    }

    void init_interrupts(uint32_t interrupt_list, uint32_t interrupt_line);
    void set_fifo_watermark(uint32_t fifo, uint32_t watermark);

    void set_interrupt_priority(IrqPriority line0_priority, IrqPriority line1_priority) {
        set_irq_priority(impl::irq0_numbers[std::to_underlying(_peripheral)], line0_priority);
        set_irq_priority(impl::irq1_numbers[std::to_underlying(_peripheral)], line1_priority);
    }

    void enable_interrupts() {
        enable_irq(impl::irq0_numbers[std::to_underlying(_peripheral)]);
        enable_irq(impl::irq1_numbers[std::to_underlying(_peripheral)]);
    }

    void disable_interrupts() {
        disable_irq(impl::irq0_numbers[std::to_underlying(_peripheral)]);
        disable_irq(impl::irq1_numbers[std::to_underlying(_peripheral)]);
    }
protected:
    void enable_clk() {
        if (_clk_enabled) return;
        __HAL_RCC_FDCAN_CLK_ENABLE();
        _clk_enabled = true;
    }
};


} // namespace can


} // namespace mcu


#endif
#endif
