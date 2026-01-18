/**
****************************(C)SWJTU_ROBOTCON2026****************************
  * @file       uart_dma_h7.c/h
  * @brief      通用 UART + DMA 接收驱动（仅适配 STM32H7 系列）
  * @note
  *             本文件实现串口空闲中断 + DMA 环形接收机制，
  *             针对 STM32H7 的 DMA 与 USART 外设寄存器差异进行了适配。
  *             注意：在 CubeMX 生成代码时，应确保 DMA 初始化在 UART 初始化之前。
  *
  * @history
  *  Version    Date          Author          Modification
  *  V1.0.0     2022-01-12     ZDYukino        First release for STM32F4
  *  V2.0.0     2025-11-06     boli235     Ported and optimized for STM32H7(DMA/USART register fix)
  *
  @verbatim
  ==============================================================================
  - 支持功能：
      • 空闲中断 + DMA 自动接收机制
      • 自动重启 DMA，防止数据丢失
      • 统一回调接口，方便多串口扩展
  - 注意事项：
      • 注意事项：HAL生成代码时，务必让DMA_init()在USART_init()之前！！！
      • 在中断回调中调用 UART_DMA_Receive_IT()
      • H7 平台需使用 USARTx->ISR / RDR 替代 SR / DR 寄存器访问
      • 仅适用于 STM32H7 平台（F4 需使用V1.0.0旧版）
  ==============================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON2026****************************
  **/

#ifndef UART_DMA_H
#define UART_DMA_H

#include "main.h"
#include "usart.h"

#define buffer_receive_length_1 80
#define buffer_receive_length_2 50
#define buffer_receive_length_3 50
#define buffer_receive_length_4 50
#define buffer_receive_length_5 50
#define buffer_receive_length_6 50
#define buffer_receive_length_7 100
#define buffer_receive_length_8 50
#define buffer_receive_length_9 50
#define buffer_receive_length_10 80

extern uint8_t buffer_receive_1[buffer_receive_length_1];//缓存数组全局定义
extern uint8_t buffer_receive_2[buffer_receive_length_2];//缓存数组全局定义
extern uint8_t buffer_receive_3[buffer_receive_length_3];//缓存数组全局定义
extern uint8_t buffer_receive_4[buffer_receive_length_4];//缓存数组全局定义
extern uint8_t buffer_receive_5[buffer_receive_length_5];//缓存数组全局定义
extern uint8_t buffer_receive_6[buffer_receive_length_6];//缓存数组全局定义
extern uint8_t buffer_receive_7[buffer_receive_length_7];//缓存数组全局定义
extern uint8_t buffer_receive_8[buffer_receive_length_8];//缓存数组全局定义
extern uint8_t buffer_receive_9[buffer_receive_length_9];//缓存数组全局定义
extern uint8_t buffer_receive_10[buffer_receive_length_10];//缓存数组全局定义
/**
  * @brief          初始化串口DMA接收
  * @param[in]      UART接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
extern void UART_DMA_Receive_init(UART_HandleTypeDef *usart, uint8_t *buffer, uint8_t length);
/**
  * @brief          串口DMA接收中断函数->放入《USER CODE BEGIN USARTX_IRQn 1》 中
  * @param[in]      UART接口
  * @param[in]      UART DMA接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
extern void UART_DMA_Receive_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t length);
#endif
