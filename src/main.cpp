#include <Arduino.h>
// NOLINTBEGIN(misc-include-cleaner)
#include <CRSFforArduino.hpp>
#include <gsl/gsl>
// NOLINTEND(misc-include-cleaner)

using namespace std;

extern void setup();
extern void loop();

namespace
{
    const auto cfa_global = make_unique<CRSFforArduino>();
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        yield();
    }

    cfa_global->begin();
}

void loop()
{
    cfa_global->update();
}
