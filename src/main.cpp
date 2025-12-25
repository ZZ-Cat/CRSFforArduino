#include <Arduino.h>
// NOLINTBEGIN(misc-include-cleaner)
#include <CRSFforArduino.hpp>
#include <gsl/gsl>
// NOLINTEND(misc-include-cleaner)

using namespace std;

extern void setup();
extern void loop();

class watchdog_timer
{
public:
    watchdog_timer() = default;
    ~watchdog_timer() = default;
    watchdog_timer(const watchdog_timer &) = delete;
    watchdog_timer(watchdog_timer &&) = delete;
    auto operator=(const watchdog_timer &) -> watchdog_timer & = delete;
    auto operator=(watchdog_timer &&) -> watchdog_timer & = delete;
    static void begin()
    {
        // Implementation for watchdog timer initialization with timeout
#if defined(__SAMD51J19A__)
        // Enable the WDT if it's disabled.
        if ((WDT->CTRLA.reg & WDT_CTRLA_ENABLE) == 0)
        {
            // Set up the watchdog timer with the specified timeout
            WDT->CONFIG.reg = WDT_CONFIG_PER_CYC32; // Example: set period to 32K cycles
            WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET_CYC16; // Example: set early warning to 16K cycles

            // Enable the watchdog timer
            WDT->CTRLA.reg |= WDT_CTRLA_ENABLE;    // Enable the watchdog timer

            // Wait for synchronization
            while (WDT->SYNCBUSY.reg & WDT_SYNCBUSY_ENABLE) {}
        }
#endif
    }
    void interrupt_enable()
    {
        // Implementation for enabling watchdog timer interrupt
#if defined(__SAMD51J19A__)
        // Enable early warning interrupt
        WDT->INTENSET.reg = WDT_INTENSET_EW; // Enable Early Warning interrupt

        // Enable WDT interrupt in NVIC, and ensure priority is set appropriately
        NVIC_SetPriority(WDT_IRQn, 0); // Set highest priority (0 is highest on ARM Cortex-M)
        NVIC_EnableIRQ(WDT_IRQn);
#endif
    }
    // void interrupt_disable()
    // {
    //     // Implementation for disabling watchdog timer interrupt
    //     #if defined(__SAMD51J19A__)
    //     // Disable early warning interrupt
    //     WDT->INTENCLR.reg = WDT_INTENSET_EW; // Disable Early Warning interrupt

    //     // Disable WDT interrupt in NVIC
    //     NVIC_DisableIRQ(WDT_IRQn);
    //     #endif
    // }
    void feed()
    {
        // Implementation for feeding (resetting) the watchdog timer
#if defined(__SAMD51J19A__)
        // Check if the interrupt flag is set
        if (interrupt_flag)
        {
            // Clear the interrupt flag
            interrupt_flag = false;

            // Clear the WDT to reset the timer
            WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;

            // Wait for synchronization
            while (WDT->SYNCBUSY.reg & WDT_SYNCBUSY_CLEAR) {}
        }
#endif
    }
    // void end()
    // {
    //     // Implementation for watchdog timer disable
    //     #if defined(__SAMD51J19A__)
    //     // Can only disable WDT if ALWAYSON is not set
    //     if ((WDT->CTRLA.reg & WDT_CTRLA_ALWAYSON) == 0)
    //     {
    //         WDT->CTRLA.reg &= ~WDT_CTRLA_ENABLE; // Disable the watchdog timer

    //         // Wait for synchronization
    //         while (WDT->SYNCBUSY.reg & WDT_SYNCBUSY_ENABLE);
    //     }
    //     #endif
    // }
    void early_warning_interrupt_handler()
    {
        // Implementation for early warning interrupt handling
#if defined(__SAMD51J19A__)
        // Check if the early warning interrupt flag is set
        if (WDT->INTFLAG.reg & WDT_INTFLAG_EW)
        {
            // Set the interrupt flag to indicate that the interrupt has occurred
            interrupt_flag = true;

            // Clear the early warning interrupt flag
            WDT->INTFLAG.reg = WDT_INTFLAG_EW;
        }
#endif
    }
private:
    // Interrupt flag to pass between ISR and main code
    volatile bool interrupt_flag = false;
};

const auto watchdog = make_unique<watchdog_timer>();
const auto cfa_global = make_unique<CRSFforArduino>();
const unsigned char RC_CHANNEL_COUNT = 16;

namespace
{
    // NOLINTBEGIN(misc-include-cleaner)
    void rc_channels_callback(const cfa_internal::control_data_t &control_data)
    {
        // Example callback implementation: print channel values to Serial
        Serial.print("RC Channel Values:[");
        Serial.print(" ch1: "); Serial.print(control_data.rc_channels.rc_channel_1);
        Serial.print(" ch2: "); Serial.print(control_data.rc_channels.rc_channel_2);
        Serial.print(" ch3: "); Serial.print(control_data.rc_channels.rc_channel_3);
        Serial.print(" ch4: "); Serial.print(control_data.rc_channels.rc_channel_4);
        Serial.print(" ch5: "); Serial.print(control_data.rc_channels.rc_channel_5);
        Serial.print(" ch6: "); Serial.print(control_data.rc_channels.rc_channel_6);
        Serial.print(" ch7: "); Serial.print(control_data.rc_channels.rc_channel_7);
        Serial.print(" ch8: "); Serial.print(control_data.rc_channels.rc_channel_8);
        // Serial.print(" ch9: "); Serial.print(control_data.rc_channels.rc_channel_9);
        // Serial.print(" ch10: "); Serial.print(control_data.rc_channels.rc_channel_10);
        // Serial.print(" ch11: "); Serial.print(control_data.rc_channels.rc_channel_11);
        // Serial.print(" ch12: "); Serial.print(control_data.rc_channels.rc_channel_12);
        // Serial.print(" ch13: "); Serial.print(control_data.rc_channels.rc_channel_13);
        // Serial.print(" ch14: "); Serial.print(control_data.rc_channels.rc_channel_14);
        // Serial.print(" ch15: "); Serial.print(control_data.rc_channels.rc_channel_15);
        // Serial.print(" ch16: "); Serial.print(control_data.rc_channels.rc_channel_16);
        Serial.println(" ]");
    }
    // NOLINTEND(misc-include-cleaner)
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        yield();
    }

#if defined(__SAMD51J19A__)
    // Catch system reset caused by watchdog timer.
    if (RSTC->RCAUSE.reg & RSTC_RCAUSE_WDT)
    {
        Serial.println(F("System reset caused by Watchdog Timer."));

        // Stop here for debugging purposes.
        while (true)
        {
            yield();
        }
    }
#endif

    watchdog->begin();
    watchdog->interrupt_enable();
    cfa_global->begin();
    cfa_global->set_rc_channels_callback(rc_channels_callback);
}

void loop()
{
    cfa_global->update();
    watchdog->feed();
}

// Watchdog Timer Interrupt Service Routine
#if defined(__SAMD51J19A__)
extern "C" void WDT_Handler()
{
    watchdog->early_warning_interrupt_handler();
}
#endif
