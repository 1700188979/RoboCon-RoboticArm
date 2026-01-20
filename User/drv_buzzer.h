#ifndef BUZZER_H
#define BUZZER_H
#include "drv_tim_h7.h"

#define MAX_PSC             1000
#define MAX_BUZZER_PWM      27000
#define MIN_BUZZER_PWM      10000
#define t_4 631
#define t_8 315
#define t_16 158
class Class_Buzzer
{
public:

    void Buzzer_on(uint16_t psc, uint16_t pwm);
    void Buzzer_off();
    void After_Initialization();
    void music(int num);

protected:

    uint16_t psc = 0;
    uint16_t pwm = MIN_BUZZER_PWM;
};

#endif
