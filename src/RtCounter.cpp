/*
  MIT License

  Copyright (c) 2020 clausgf@github

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "RtCounter.h"


#ifndef ARDUINO_ARCH_SAMD
#error "This library supports SAMD MCUs only - port to more MCU are welcome :-)"
#endif

// **************************************************************************

class RtCounter RtCounter;

voidFuncPtr RtCounter::_OverflowCallback = NULL;
voidFuncPtr RtCounter::_CompareMatchCallback = NULL;

const uint32_t RtCounter::DELTA_TICKS_MIN;
const uint32_t RtCounter::TICKS_PER_S;

// **************************************************************************

RtCounter::RtCounter()
{
  _enabled = false;
}

// **************************************************************************

bool RtCounter::begin()
{
  // do not initialized the RTC class/peripheral twice
  if (_enabled)
  {
    return false;
  }

  // turn on the clock for the RTC's digital interface
  PM->APBAMASK.reg |= PM_APBAMASK_RTC;

  // Configure the XOSC32K oscillator:
  // The "framework-arduinosam" core configures the SAM21D XOSC32K as enabled,
  // using Generic Clock Generator 1. There is no divider configured.
  // Thus we just have to reconfigure the oscillator itself to remain active
  // in deep in standby mode.
#ifdef CRYSTALLESS
# error "Crystalless operation not supported"
#else
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_ONDEMAND |   // new
                         SYSCTRL_XOSC32K_RUNSTDBY |   // new and important!
                         SYSCTRL_XOSC32K_EN32K |      // same as framework
                         SYSCTRL_XOSC32K_XTALEN |     // same as framework
                         SYSCTRL_XOSC32K_STARTUP(6) | // same as framework
                         SYSCTRL_XOSC32K_ENABLE;      // same as framework
#endif

  
  GCLK->CLKCTRL.reg = (uint32_t)((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos)));
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // reset and disable the RTC peripheral
  RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_SWRST;
  sync();

  // configure and enable the RTC peripheral
  RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_PRESCALER_DIV1 | RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_ENABLE;
  sync();

  _enabled = true;

  // prepare interrupts
  NVIC_EnableIRQ(RTC_IRQn);
  NVIC_SetPriority(RTC_IRQn, 0x00);
  detachCompareMatchInterrupt();
  detachOverflowInterrupt();

  return true;
}

// **************************************************************************

void RtCounter::end()
{
  // disable interrupts
  detachCompareMatchInterrupt();
  detachOverflowInterrupt();
  NVIC_DisableIRQ(RTC_IRQn);

  // reset and disable the RTC peripheral
  RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_SWRST;
  sync();

  // reconfigure the XOSC32K oscillator
#ifdef CRYSTALLESS
# error "Crystalless operation not supported"
#else
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_EN32K |      // same as framework
                         SYSCTRL_XOSC32K_XTALEN |     // same as framework
                         SYSCTRL_XOSC32K_STARTUP(6) | // same as framework
                         SYSCTRL_XOSC32K_ENABLE;      // same as framework
#endif

  // turn off the clock for the RTC peripheral's digitial interface

  _enabled = false;
}

// **************************************************************************

/**
 * Interrupt handler:
 * - Overflow increments further 32 bit counter
 * - Compare calls attached handler function (if registered)
 * The interrupt flags are cleared on exit.
 */
void RTC_Handler(void)
{
  // handle overflow interrupt
  if (RTC->MODE0.INTFLAG.bit.OVF)
  {
    if (RtCounter::_OverflowCallback != NULL)
    {
      RtCounter::_OverflowCallback();
    }
    RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_OVF;
  }

  // handle compare match interrupt
  if (RTC->MODE0.INTFLAG.bit.CMP0)
  {
    if (RtCounter::_CompareMatchCallback != NULL)
    {
      RtCounter::_CompareMatchCallback();
    }
    RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;
  }
}

// **************************************************************************

void RtCounter::attachOverflowInterrupt(void (*callback)(void))
{
  _OverflowCallback = callback;
  if (_enabled)
  {
    RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_OVF;
    sync();
  }
}

void RtCounter::RtCounter::detachOverflowInterrupt()
{
  _OverflowCallback = NULL;
  if (_enabled)
  {
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_OVF;
    sync();
  }
}

// **************************************************************************

void RtCounter::attachCompareMatchInterrupt(uint32_t deltaTicks, voidFuncPtr callback)
{
  _CompareMatchCallback = callback;
  if (_enabled)
  {
    // The pseudocode of this function might be 
    //   compareReg=counterReg+deltaTicks
    // This implementation tries to avoid the problem of missed comparisons
    // and interrupts for small deltaTicks due to delays between 
    // reading counterReg and writing compareReg.
    // This implemenation is not bulletproof - who knows a better one?

    // temporarily buy time for the next operations by setting compareReg to 
    // the far future
    RTC->MODE0.COMP[0].bit.COMP = getTicks() - 1;
    sync();

    // activate the compare match interrupt
    RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
    sync();

    // Disable interrupts because these might introduce an arbitrary delay.
    __disable_irq();

    // finally write the correct compareReg value
    RTC->MODE0.COMP[0].bit.COMP = getTicks() + deltaTicks;
    sync();

    __enable_irq();
  }
}

void RtCounter::detachCompareMatchInterrupt()
{
  _CompareMatchCallback = NULL;
  if (_enabled)
  {
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
    sync();
  }
}

// **************************************************************************

void RtCounter::standbyModeTicks(uint32_t deltaTicks)
{
  if (_enabled)
  {
    if (deltaTicks != 0)
    {
      _CompareMatchCallback = NULL;

      // temporarily buy time for the next operations by setting compareReg to 
      // the far future
      RTC->MODE0.COMP[0].bit.COMP = getTicks() - 1;
      sync();

      // activate the compare match interrupt
      RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
      sync();

      // Disable interrupts - we won't enter standby if an IRQ is pending
      __disable_irq();

      // finally write the correct compareReg value
      RTC->MODE0.COMP[0].bit.COMP = getTicks() + deltaTicks;
      sync();
    }

    if (deltaTicks >= DELTA_TICKS_MIN)
    {
      // Enter standby mode. This may cause problems when connected via native USB.
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    	PM->SLEEP.reg = PM_SLEEP_IDLE_APB; // stop CPU, AHB and APB clock domains on WFI
      __DSB();
      __WFI();
    }

    if (deltaTicks != 0)
    {
      __enable_irq();
      detachCompareMatchInterrupt();
    }
  }
}


void RtCounter::standbyModeMs(uint32_t deltaMs)
{
  uint32_t deltaTicks = msToTicks(deltaMs);
  standbyModeTicks(deltaTicks);
}


uint32_t RtCounter::getTicks()
{
  uint32_t ticks = 0;

  if (_enabled) {
    // synchronize counter value for reading
    RTC->MODE0.READREQ.reg = RTC_READREQ_RREQ;
    sync();

    // read counter
    ticks = RTC->MODE0.COUNT.reg;
  }

  return ticks;
}


// **************************************************************************
// Utility functions
// **************************************************************************

inline bool RtCounter::isSyncing()
{
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

inline void RtCounter::sync()
{
  while ((RTC->MODE0.STATUS.bit.SYNCBUSY) != 0)
    ;
}
