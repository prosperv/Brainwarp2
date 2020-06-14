#include <Arduino.h>
#include <RA4051.h>

class Fast4051
{
public:
  Fast4051()
  {
    DDRD = 0b11110000;
    PORTD = 0b00000000;
  };
  // Set the multiplexer pin to "pinToSet"
  void setPin(uint8_t pinToSet)
  {
    uint8_t out = pinToSet << 4;
    uint8_t enableMask = 0b10000000;
    out |= (enableMask & PORTD);
    //Write to the port in one go to avoid transistion errors
    PORTD = out;
    _currentPin = pinToSet;
  };
  // Get what pin is currently set
  int getCurrentPin()
  {
    return _currentPin;
  };
  uint8_t getPort()
  {
    return PORTD;
  };
  // Enable the 4051
  void on()
  {
    bitClear(PORTD, 1);
  };
  // Disable the 4051
  void off()
  {
    bitSet(PORTD, 1);
  };

protected:
  int _s0 = 2;
  int _s1 = 3;
  int _s2 = 4;
  int _e = 1;
  int _enableBit;
  int _currentPin;

private:
};

Fast4051 fastMux;
void fastMux_setup()
{
  fastMux.setPin(0);
  // Turn on the chip's outs
  fastMux.on();
}

void fastMux_loop()
{
  // Cycle through the outputs
  for (size_t i = 0; i < 8; i++)
  {
    Serial.println(i);
    fastMux.setPin(i);
    auto value = fastMux.getPort();
    Serial.println(value, BIN);
    Serial.println();
    delay(700);
  }
  // Turn off the outputs
  fastMux.off();
  auto value = fastMux.getPort();
  Serial.println(value, BIN);
  Serial.println();
  delay(700);
  // Turn them back on and loop
  fastMux.on();
}

RA4051 mux(12, 11, 10);
void RA4051_setup()
{
  // Set it to Pin 0
  mux.setPin(0);
  // Set the enable pin
  mux.setEnablePin(9);
  // Turn on the chip's outs
  mux.on();
}
void RA4051_loop()
{
  // Cycle through the outputs
  for (size_t i = 0; i < 8; i++)
  {
    Serial.println(i);
    /* 
    When transistioning between multiple bit changes, 
    a different switch will be active as the library 
    sets each bit at a time. So if we use this library 
    we will need to use the disable and enable the mux 
    between each switch.
    Example:
    From 1 to 2
    0001
    0010
    In will realistically go
    0001
    0000
    0010
    */
    mux.setPin(i);
    delay(1000);
  }
  // Turn off the outputs
  mux.off();
  delay(1000);
  // Turn them back on and loop
  mux.on();
}

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
  // RA4051_setup();
  fastMux_setup();
  Serial.println("Starting loop.");
}

void loop()
{
  // RA4051_loop();
  fastMux_loop();
}
