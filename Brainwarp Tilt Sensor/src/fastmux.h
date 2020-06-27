#pragma once

// Support both the Uno and the Attiny85 boards as they will be some slight differences.
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
const uint8_t ENABLE_PORT_MASK = 0b00000010;
const uint8_t SWITCH_PORT_MASK = 0b00011100;
const uint8_t ENABLE_OFFSET = 3;
const int SWITCH_OFFSET = 2;
#endif

class Fast4051
{
public:
    Fast4051()
    {
        begin();
    };

    void begin()
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
