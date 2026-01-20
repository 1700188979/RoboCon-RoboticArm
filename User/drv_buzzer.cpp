#include "drv_buzzer.h"

void Class_Buzzer :: Buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm);
}

void Class_Buzzer :: Buzzer_off()
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
}
void Class_Buzzer::music(int num)
{
    switch (num)
    {
        case 0: Buzzer_off();
        case 1: Buzzer_on(21, 2700); break;//Cb
        case 2: Buzzer_on(20, 2700); break;//Db
        case 3: Buzzer_on(19, 2700); break;//Eb
        case 4: Buzzer_on(18, 2700); break;//Fb
        case 5: Buzzer_on(17, 2700); break;//Gb
        case 6: Buzzer_on(16, 2700); break;//Ab
        case 7: Buzzer_on(15, 2700); break;//Bb
        case 11: Buzzer_on(14, 2700); break;//C
        case 12: Buzzer_on(13, 2700); break;//D
        case 13: Buzzer_on(12, 2700); break;//E
        case 14: Buzzer_on(11, 2700); break;//F
        case 15: Buzzer_on(10, 2700); break;//G
        case 16: Buzzer_on(9, 2700); break;//A
        case 17: Buzzer_on(8, 2700); break;//B
        case 21: Buzzer_on(7, 2700); break;//C#
        case 22: Buzzer_on(6, 2700); break;//D#
        case 23: Buzzer_on(5, 2700); break;//E#
        case 24: Buzzer_on(4, 2700); break;//F#
        case 25: Buzzer_on(3, 2700); break;//G#
        case 26: Buzzer_on(2, 2700); break;//A#
        case 27: Buzzer_on(1, 2700); break;//B#
    }
}
void Class_Buzzer :: After_Initialization()
{
    HAL_Delay(150);
    Buzzer_on(4, 2700);
    HAL_Delay(250);

    Buzzer_off();
}

