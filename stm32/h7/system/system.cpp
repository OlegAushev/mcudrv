#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/system/system.h>


namespace mcu {


void init_hal() {
    HAL_Init();
}


void reset_device() {
    HAL_NVIC_SystemReset();
}


void fatal_error(const char* hint, int code) {
    emb::fatal_error(hint, code);
}


} // namespace mcu


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    char buf[256] = {};
    snprintf(buf, 255, "assertion failed at %s, line %lu", reinterpret_cast<char*>(file), line);
    emb::fatal_error(buf, static_cast<int>(line));
}
#endif // USE_FULL_ASSERT


#endif
#endif
