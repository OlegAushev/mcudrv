#ifdef STM32H7xx

#include <mculib_stm32/f4/dma/dma.h>


extern "C" void DMA1_Stream0_IRQHandler(void) {
    using namespace mcu::dma;
    HAL_DMA_IRQHandler(StreamController::instance(Stream::dma1_stream0)->peripheral_handle());
}


extern "C" void DMA1_Stream1_IRQHandler(void) {
    using namespace mcu::dma;
    HAL_DMA_IRQHandler(StreamController::instance(Stream::dma1_stream1)->peripheral_handle());
}


extern "C" void DMA1_Stream2_IRQHandler(void) {
    using namespace mcu::dma;
    HAL_DMA_IRQHandler(StreamController::instance(Stream::dma1_stream2)->peripheral_handle());
}


extern "C" void DMA1_Stream3_IRQHandler(void) {
    using namespace mcu::dma;
    HAL_DMA_IRQHandler(StreamController::instance(Stream::dma1_stream3)->peripheral_handle());
}

#endif

