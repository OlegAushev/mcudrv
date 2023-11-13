#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mculib_stm32/h7/gpio/gpio.h>


extern "C" void EXTI0_IRQHandler() {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    mcu::gpio::Input::on_interrupt[0]();
}


extern "C" void EXTI1_IRQHandler() {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
    mcu::gpio::Input::on_interrupt[1]();
}


extern "C" void EXTI2_IRQHandler() {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    mcu::gpio::Input::on_interrupt[2]();
}


extern "C" void EXTI3_IRQHandler() {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    mcu::gpio::Input::on_interrupt[3]();
}


extern "C" void EXTI4_IRQHandler() {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    mcu::gpio::Input::on_interrupt[4]();
}


extern "C" void EXTI9_5_IRQHandler() {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);	
}


extern "C" void EXTI15_10_IRQHandler() {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    size_t pinNo = POSITION_VAL(GPIO_Pin);
    assert_param(pinNo <= 15);
    mcu::gpio::Input::on_interrupt[pinNo]();
}


#endif
#endif
