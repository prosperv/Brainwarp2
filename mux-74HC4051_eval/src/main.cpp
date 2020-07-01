#include <Arduino.h>
#include <RA4051.h>

#ifdef _AVR_IOM328P_H_
#define ENABLE_PIN 7
RA4051 mux(6, 5, 4);
#elif ARDUINO_attiny3217
#define ENABLE_PIN 10
RA4051 mux(11, 12, 13);
#endif

void printPinInfo()
{
  for (int pin = 0; pin <= 13; pin++)
  {
    Serial.print("Pin: ");
    Serial.println(pin);
    Serial.print("  Port: ");
    uint8_t port = digitalPinToPort(pin);
    Serial.println(port);
    Serial.print("  PortReg: ");
    volatile uint8_t *out = portOutputRegister(port);
    Serial.println(*out);
    Serial.print("  Bit: ");
    uint8_t bit = digitalPinToBitMask(pin);
    Serial.println(bit);
  }
}
void setup()
{
  Serial.begin(115200);
  // printPinInfo();

  // Set it to Pin 0
  mux.setPin(0);
  // Set the enable pin
  mux.setEnablePin(ENABLE_PIN);
  // Turn on the chip's outs
  mux.on();

  Serial.println("Starting loop.");
}

void loop()
{ // Cycle through the outputs
  for (size_t i = 0; i < 8; i++)
  {
    Serial.println(i);
    // When transistioning between multiple bit changes, a different switch will be active as the library sets each bit at a time. So if we use this library we will need to use the disable and enable the mux between each switch.
    mux.setPin(i);
    delay(1000);
  }
  // Turn off the outputs
  mux.off();
  delay(1000);
  // Turn them back on and loop
  mux.on();
}
