#include <Arduino.h>

#ifdef _AVR_IOM328P_H_
#define PORT_DIRECTION DDRD
#define PORT_OUTPUT PORTD
const uint8_t ENABLE_PORT_MASK = 0b10000000;
const uint8_t SWITCH_PORT_MASK = 0b01110000;
const uint8_t ENABLE_OFFSET = 7;
const int SWITCH_OFFSET = 4;
#elif ARDUINO_attiny3217
#define PORT_DIRECTION VPORTA_DIR
#define PORT_OUTPUT VPORTA_OUT
const uint8_t ENABLE_PORT_MASK = 0b00010000;
const uint8_t SWITCH_PORT_MASK = 0b00001110;
const uint8_t ENABLE_OFFSET = 3;
const int SWITCH_OFFSET = 1;
#endif

class Fast4051
{
public:
  Fast4051()
  {
    //Ensure pins are set to output.
#ifdef _AVR_IOM328P_H_
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
#elif ARDUINO_attiny3217
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
#endif
    PORT_DIRECTION |= ENABLE_PORT_MASK | SWITCH_PORT_MASK;
    PORT_OUTPUT = 0b00000000;
  };
  // Set the multiplexer pin to "pinToSet"
  void setPin(uint8_t pinToSet)
  {
    // Set value to the correct bits
    uint8_t out = pinToSet << SWITCH_OFFSET;
    // Ensure we preserve our enable value
    out |= (ENABLE_PORT_MASK & PORT_OUTPUT);

    //Write to the port in one go to avoid transistion errors. No longer really matters.
    PORT_OUTPUT = out;
    Serial.println(out, BIN);
    _currentPin = pinToSet;
  };
  // Get what pin is currently set
  int getCurrentPin()
  {
    return _currentPin;
  };
  uint8_t getPort()
  {
    return PORT_OUTPUT;
  };
  // Enable the 4051
  void on()
  {
    PORT_OUTPUT &= ~ENABLE_PORT_MASK;
  };
  // Disable the 4051
  void off()
  {
    PORT_OUTPUT |= ENABLE_PORT_MASK;
  };

protected:
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

  auto value = fastMux.getPort();
  Serial.println(value, BIN);
  Serial.println();
  delay(700);
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
    Serial.println();
  }
}

void mux_setup()
{
  Serial.begin(115200);
  printPinInfo();
  fastMux_setup();
  Serial.println("Starting loop.");
}

void mux_loop()
{
  Serial.println("No enable");
  // Cycle through the outputs
  for (size_t i = 0; i < 8; i++)
  {
    delay(1000);
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
    In reality it will in steps
    0001
    0000
    0010
    */
    fastMux.on();
    // Serial.println(i);
    fastMux.setPin(i);
  }
  fastMux.off();
  delay(1000);

  Serial.println("With enable");
  for (size_t i = 0; i < 8; i++)
  {
    delay(1000);
    // Serial.println(i);
    fastMux.off();
    fastMux.setPin(i);
    fastMux.on();
  }
  fastMux.off();

  delay(1000);
}