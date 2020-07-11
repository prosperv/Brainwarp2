#include <Arduino.h>

#define POWERONOFF_PIN 15
volatile bool sleep = false;

#ifdef _AVR_ATTINY3217_H_INCLUDED

void powerOnOff_interruptHandler()
{
    sleep = !digitalRead(POWERONOFF_PIN);
}

static void enterIdle()
{
    SLPCTRL.CTRLA = SLEEP_MODE_IDLE | SLPCTRL_SEN_bm;
    __builtin_avr_sleep();
};

static void enterStandby()
{
    SLPCTRL.CTRLA = SLEEP_MODE_STANDBY | SLPCTRL_SEN_bm;
    __builtin_avr_sleep();
};

static void enterPowerDown()
{
    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN | SLPCTRL_SEN_bm;
    __builtin_avr_sleep();
};

#else
static void enterIdle(){};
static void enterStandby(){};
static void enterPowerDown(){};
#endif