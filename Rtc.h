/**
 * Rtc library for Atmel/Microchip SAMD based Arduinos' RTC
 * peripherals.
 * This library uses the RTC peripheral in mode 0 as a 32 bit counter 
 * instead of clock/calendar mode 2 (Year-Month-Day-...) in the RTCZero
 * library. Some microcontroller applications do not require the 
 * calendar date but benefit from the simplified timer handling in
 * this library.
 */

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

#ifndef Rtc_H
#define Rtc_H

#include "Arduino.h"


/**
 * This class uses the SAMD's RTC peripheral. In contrast to the RTCZero 
 * library which uses clock/calendar mode (mode 2) of the RTC, this class
 * employs RTC mode 0 which implements a free running 32 bit counter. This better
 * suits such embedded applications which measure (relative) time differences 
 * instead of keeping track of absolute wall clock time/date. It was written
 * to enable LORAWAN libraries like LMIC to use this clock as a timer which
 * continues running in sleep mode.
 * 
 * The counter runs at 32768 Hz, i.e. it wraps around after ca. 1.5 days.
 *
 * This class was tested on a SAMD21 controller with external 32.768 kHz
 * quartz (standard configuration on Adafruit Feather M0).
 */
class Rtc
{
public:

  /// Constructor
  Rtc();

  /// Enable the RTC class and peripheral
  bool begin();

  /// Disable the RTC class and peripheral
  void end();

  /// Return whether the class has been initialized by calling begin().
  bool isConfigured()
  {
    return _enabled;
  }

  /// Attach the overflow interrupt handler and enable overflow interrupts.
  void attachOverflowInterrupt(void (*callback)(void));

  /// Detach the overflow interrupt handler.
  void detachOverflowInterrupt();

  /**
   * Attach (and enable) a compare match interrupt handler 
   * which will be called after deltaTicks seconds.
   * 
   * Warning: This function temporarily disables and re-enables interrupts!
   */
  void attachCompareMatchInterrupt(uint32_t deltaTicks, void (*callback)(void));

  /// Detach the compare match interrupt handler.
  void detachCompareMatchInterrupt();

  /**
   * Put the system into standby mode, wake up after deltaTicks or
   * the next interrupt (i.e. the system might wake up earlier).
   * 
   * If deltaTicks is zero, just put the system to sleep without
   * timeout. Interrupts installed prior to calling this function
   * will still wake up the system.
   * 
   * If deltaTicks is differenct from zero, a compareMatch interrupt
   * is installed prior to putting the system into stand by mode.
   * If deltaTicks != 0, this function will override any timeouts
   * and callbacks from attachCompareMatchInterrupt.
   * 
   * Warning: This function temporarily disables and re-enables interrupts!
   * 
   * Warning: This function does not take care of a native USB port 
   * emulating a serial interface - an active USB serial connection
   * might be disturbed by this function.
   */
  void standbyModeTicks(uint32_t deltaTicks = 0);

  /**
   * Put the system into standby mode, wake up after deltaTicks or
   * the next interrupt (i.e. the system might wake up earlier).
   * 
   * See standbyModeTicks for details.
   */
  void standbyModeMs(uint32_t deltaMs = 0);

  /// Return the current time in ticks.
  uint32_t getTicks();

  static void (*_OverflowCallback)();
  static void (*_CompareMatchCallback)();

  /**
   * Miminum ticks to put the system to sleep in standbyMode
   */
  static const uint32_t DELTA_TICKS_MIN = 5;
  static const uint32_t TICKS_PER_S = 32768;

  inline uint32_t msToTicks(uint32_t ms)
  {
    const uint64_t ticks1000 = (uint64_t)ms * (uint64_t)32768;
    const uint32_t ticks = ticks1000 / (uint64_t)1000;
    return ticks;
  }

  inline uint32_t sToTicks(uint32_t s)
  {
    return s * 32768;
  }

private:
  bool _enabled;

  /// return true if the RTC is not synchronized yet
  bool isSyncing();

  /// block until the RTC is synchronized
  void sync();
};

extern class Rtc Rtc;

#endif // Rtc_H
