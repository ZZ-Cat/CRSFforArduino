#include <Arduino.h>
// NOLINTBEGIN(misc-include-cleaner)
#include <CRSFforArduino.hpp>
#include <gsl/gsl>
// NOLINTEND(misc-include-cleaner)

extern void setup();
extern void loop();

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
namespace
{
    gsl::owner<CRSFforArduino *> cfa_global = nullptr;
} // namespace
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        yield();
    }

    cfa_global = gsl::owner<CRSFforArduino *>(new CRSFforArduino());
    cfa_global->begin();
}

void loop()
{
    cfa_global->update();

    // Just to keep the example simple, we won't do anything else here.
    // while (true)
    // {
    //     yield();
    // }
}
