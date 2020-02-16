# Rtc

Arduino support for the SAMD RTC peripheral in 32 bit counter mode Rtc library for Atmel/Microchip SAMD based Arduinos' RTC peripherals.

This library uses the RTC peripheral in mode 0 as a 32 bit counter instead of clock/calendar mode 2 (Year-Month-Day-...) as seen in the RTCZero library. Some microcontroller applications do not require the calendar date but benefit from the simplified timer handling in this library.

## Usage

Usage example:
```c++
// initialization, usually called once in setup()
Rtc.begin();

// this code might appear somewhere else
// (1) get the current time in ticks (1/32768 s)
uint32_t startSleepTimeInTicks = Rtc.getTicks();
// (2) sleep 1000 milliseconds
Rtc.standbyModeMs(1000);
// (3) document how much sleep we got
Serial.print("I slept ");
Serial.print(Rtc.getTicks() - startSleepTimeInTicks);
Serial.print(" ticks. One tick is 1/32768 seconds.");
```

Full documentation is found in the header file.
