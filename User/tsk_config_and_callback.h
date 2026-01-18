/**
* @file tsk_config_and_callback.h
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* Exported macros -----------------------------------------------------------*/

//遥控器宏定义
#define CE_24L01_Pin GPIO_PIN_0
#define CE_24L01_GPIO_Port GPIOI
#define IRQ_24L01_Pin GPIO_PIN_12
#define IRQ_24L01_GPIO_Port GPIOH
#define CSN_24L01_Pin GPIO_PIN_11
#define CSN_24L01_GPIO_Port GPIOH
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOG
/* Exported types ------------------------------------------------------------*/

//遥控器外部变量
extern int c_adc;
extern int c_adc1;
extern int c_adc2;
extern int c_adc3;
extern char button[8];//三个按键,分别控制R1转至R2,R1转至篮筐
extern char last_button[8];//三个按键

extern float chassis_vmax;




/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

    void Task_Init();
    void H7_TIM_CALLBACK(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif

#endif

