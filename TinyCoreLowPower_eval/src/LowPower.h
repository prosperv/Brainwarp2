#include <Arduino.h>

#ifdef _AVR_ATTINY3217_H_INCLUDED
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
#endif