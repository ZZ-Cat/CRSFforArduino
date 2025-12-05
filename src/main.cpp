#include <Arduino.h>
// NOLINTBEGIN(misc-include-cleaner)
#include <CRSFforArduino.hpp>
#include <gsl/gsl>
// NOLINTEND(misc-include-cleaner)

// namespace my_namespace
// {
//     class class_a_base
//     {
//     public:
//         // Default constructor.
//         class_a_base() = default;

//         // Copy constructor, copy assignment operator, move constructor, and move assignment operator.
//         class_a_base(const class_a_base&) = default;
//         auto operator=(const class_a_base &) -> class_a_base & = default;
//         class_a_base(class_a_base &&) = default;
//         auto operator=(class_a_base &&) -> class_a_base & = default;

//         virtual ~class_a_base() = default;

//         virtual void do_something() = 0;
//     };

//     class class_a_derived : private class_a_base
//     {
//     public:
//         void do_something() override
//         {
//             Serial.println("Derived class doing something!");
//         }

//     };
// } // namespace my_namespace

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
        // delay(100);
    }

    // Serial.println("Hello, World!");

    // my_namespace::class_a_derived obj;
    // obj.do_something();

    cfa_global = gsl::owner<CRSFforArduino *>(new CRSFforArduino());
}

void loop()
{}
