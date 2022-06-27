#include "wiring_private.h"

static inline void wait_for_sync(Tc *const hw) {
  while (hw->COUNT8.SYNCBUSY.reg > 0) {}
}

bool clockSetup() {
  Tc *_hw = TC3;
  uint32_t gclk_id = TC3_GCLK_ID;
  uint32_t clock_prescaler = TC_CTRLA_PRESCALER(0); // no prescaling
  uint32_t counter_size = TC_CTRLA_MODE_COUNT8;
  uint32_t wave_generation = TC_WAVE_WAVEGEN_MFRQ;
  uint32_t pin_out = 6;
  uint32_t pin_mux = MUX_PA18E_TC3_WO0;
  uint8_t compare_value = 2;

  
  if ((_hw->COUNT8.CTRLA.reg & TC_CTRLA_SWRST) || /* We are in the middle of a reset. Abort. */
      (_hw->COUNT8.STATUS.reg & TC_STATUS_SLAVE) || /* Module is used as a slave */
      (_hw->COUNT8.CTRLA.reg & TC_CTRLA_ENABLE) ) { /* Module must be disabled before initialization. Abort. */
    return false;
  }

  pinPeripheral(pin_out, (EPioType)pin_mux);

  GCLK->PCHCTRL[gclk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK1_Val |
      (1 << GCLK_PCHCTRL_CHEN_Pos); // use GCLK1 to get 48MHz on SAMD51

  /* Write configuration to register */
  wait_for_sync(_hw);
  
  _hw->COUNT8.CTRLA.reg = counter_size | clock_prescaler;

  wait_for_sync(_hw);
  
  _hw->COUNT8.WAVE.reg = wave_generation;

  wait_for_sync(_hw);
  
  _hw->COUNT16.CC[0].reg = compare_value;

  wait_for_sync(_hw);

  while (_hw->COUNT8.SYNCBUSY.bit.ENABLE)
    ;

  _hw->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;

  return true;
}
