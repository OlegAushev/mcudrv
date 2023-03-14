#pragma once


#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib_c28x/array.h>
#include <emblib_c28x/core.h>


namespace mcu {

enum CapModule {
	CAP1,
	CAP2,
	CAP3,
	CAP4,
	CAP5,
	CAP6
};


template <unsigned int ChannelCount>
struct CapConfig {
	CapModule module[ChannelCount];
	mcu::gpio::Input inputPin[ChannelCount];
};


namespace impl {

template <unsigned int ChannelCount>
struct CapModuleImpl {
	CapModule instance[ChannelCount];
	uint32_t base[ChannelCount];
	XBAR_InputNum xbarInput[ChannelCount];
	uint32_t pieIntNum[ChannelCount];
};


extern const uint32_t capBases[6];
extern const XBAR_InputNum capXbarInputs[6];
extern const uint32_t capPieIntNums[6];

} // namespace impl


template <unsigned int ChannelCount>
class Cap {
private:
	impl::CapModuleImpl<ChannelCount> m_module;
private:
	Cap(const Cap& other);			// no copy constructor
	Cap& operator=(const Cap& other);	// no copy assignment operator
public:
	Cap(const CapConfig<ChannelCount>& conf) {
		EMB_STATIC_ASSERT(ChannelCount > 0);
		EMB_STATIC_ASSERT(ChannelCount <= 6);

		// XBAR setup
		for (size_t i = 0; i < ChannelCount; ++i) {
			m_module.instance[i] = conf.module[i];
			m_module.base[i] = impl::capBases[conf.module[i]];
			m_module.xbarInput[i] = impl::capXbarInputs[conf.module[i]];
			m_module.pieIntNum[i] = impl::capPieIntNums[conf.module[i]];
			XBAR_setInputPin(m_module.xbarInput[i], conf.inputPin[i].config().no);
		}

		/* ECAP setup */
		for (size_t i = 0; i < ChannelCount; ++i) {
			ECAP_disableInterrupt(m_module.base[i],
								  (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
								   ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
								   ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
								   ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
								   ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
								   ECAP_ISR_SOURCE_COUNTER_PERIOD   |
								   ECAP_ISR_SOURCE_COUNTER_COMPARE));
			ECAP_clearInterrupt(m_module.base[i],
								(ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
								 ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
								 ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
								 ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
								 ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
								 ECAP_ISR_SOURCE_COUNTER_PERIOD   |
								 ECAP_ISR_SOURCE_COUNTER_COMPARE));

			ECAP_disableTimeStampCapture(m_module.base[i]);

			ECAP_stopCounter(m_module.base[i]);
			ECAP_enableCaptureMode(m_module.base[i]);
			ECAP_setCaptureMode(m_module.base[i], ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_2);


			ECAP_setEventPolarity(m_module.base[i], ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
			ECAP_setEventPolarity(m_module.base[i], ECAP_EVENT_2, ECAP_EVNT_FALLING_EDGE);

			ECAP_enableCounterResetOnEvent(m_module.base[i], ECAP_EVENT_1);
			ECAP_enableCounterResetOnEvent(m_module.base[i], ECAP_EVENT_2);

			ECAP_setSyncOutMode(m_module.base[i], ECAP_SYNC_OUT_DISABLED);
			ECAP_startCounter(m_module.base[i]);
			ECAP_enableTimeStampCapture(m_module.base[i]);
			ECAP_reArm(m_module.base[i]);
		}
	}

	void rearm() const {
		for (int i = 0; i < ChannelCount; ++i) {
			ECAP_reArm(m_module.base[i]);
		}
	}

	void enableInterrupts() const {
		for (int i = 0; i < ChannelCount; ++i) {
			ECAP_enableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_CAPTURE_EVENT_1);
			ECAP_enableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
			ECAP_enableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_COUNTER_OVERFLOW);
		}
	}

	void disableInterrupts() const {
		for (int i = 0; i < ChannelCount; ++i) {
			ECAP_disableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_CAPTURE_EVENT_1);
			ECAP_disableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
			ECAP_disableInterrupt(m_module.base[i], ECAP_ISR_SOURCE_COUNTER_OVERFLOW);
		}
	}

	void registerInterruptHandler(CapModule module, void (*handler)(void)) const {
		for (int i = 0; i < ChannelCount; ++i) {
			if (m_module.instance[i] == module) {
				Interrupt_register(m_module.pieIntNum[i], handler);
				Interrupt_enable(m_module.pieIntNum[i]);
				return;
			}
		}
	}
};

} // namespace mcu

