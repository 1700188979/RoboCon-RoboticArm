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

#include "main.h"
#include "usart.h"
#include "UART_DMA_h7.h"
#include "string.h"
#include "vofa.h"
#include "deta10.h"
#include "visual_interact.h"

/*缓存数组预定义*/
uint8_t buffer_receive_1[buffer_receive_length_1];
uint8_t buffer_receive_2[buffer_receive_length_2];
uint8_t buffer_receive_3[buffer_receive_length_3];
uint8_t buffer_receive_4[buffer_receive_length_4];
uint8_t buffer_receive_5[buffer_receive_length_5];
uint8_t buffer_receive_6[buffer_receive_length_6];
uint8_t buffer_receive_7[buffer_receive_length_7];
uint8_t buffer_receive_8[buffer_receive_length_8];
uint8_t buffer_receive_9[buffer_receive_length_9];
uint8_t buffer_receive_10[buffer_receive_length_10];
/**
  * @brief          UART1-10中断接收服务函数
  * @param[in]      接收数组
  * @param[in]      长度值【0-128】
  * @retval         none
  */
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART2_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART3_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART4_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART5_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART6_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART7_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART8_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART9_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART10_Receive_Serve(uint8_t *buffer, uint8_t length);

/**
  * @brief          初始化串口DMA接收
  * @param[in]      UART接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
void UART_DMA_Receive_init(UART_HandleTypeDef *usart, uint8_t *buffer, uint8_t length)
{
  __HAL_UART_ENABLE_IT(usart,UART_IT_IDLE);
  HAL_UART_Receive_DMA(usart,buffer,length);//打开DMA接收
}
/**
  * @brief          串口DMA接收中断函数->放入《USER CODE BEGIN USARTX_IRQn 1》 中
  * @param[in]      UART接口
  * @param[in]      UART DMA接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
// void UART_DMA_Receive_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t length)
// {
//  if(__HAL_UART_GET_FLAG(usart, UART_FLAG_IDLE))
//  {
//      __HAL_UART_CLEAR_IDLEFLAG(usart);
//      HAL_UART_DMAStop(usart);
//      uint8_t real_length = length - (uint8_t)(((DMA_Stream_TypeDef *)DMA->Instance)->NDTR);
//
//       if(usart == &huart1) UART1_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart2) UART2_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart3) UART3_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart4) UART4_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart5) UART5_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart6) UART6_Receive_Serve(buffer, real_length);//选择解码程序
//       if(usart == &huart7) UART7_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart8) UART8_Receive_Serve(buffer, real_length);//选择解码程序
//       // if(usart == &huart9) UART9_Receive_Serve(buffer, real_length);//选择解码程序
//       if(usart == &huart10) UART10_Receive_Serve(buffer, real_length);//选择解码程序
// 	  HAL_UART_Receive_DMA(usart, buffer, length);//重新打开DMA接收
//  }
//}
void UART_DMA_Receive_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t length)
{
    if(__HAL_UART_GET_FLAG(usart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(usart);

        HAL_UART_DMAStop(usart);

        uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(DMA);
        uint16_t real_length = length - dma_remaining;

        if(real_length > 0)
        {
            if(usart == &huart1) UART1_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart2) UART2_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart3) UART3_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart4) UART4_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart5) UART5_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart6) UART6_Receive_Serve(buffer, real_length);//选择解码程序
            if(usart == &huart7) UART7_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart8) UART8_Receive_Serve(buffer, real_length);//选择解码程序
            // if(usart == &huart9) UART9_Receive_Serve(buffer, real_length);//选择解码程序
            if(usart == &huart10) UART10_Receive_Serve(buffer, real_length);//选择解码程序

        }
        HAL_UART_Receive_DMA(usart, buffer, length);//重新打开DMA接收
    }
}

/**
  * @brief          串口异常的处理
  * @param[in]      UART接口
  * @retval         none
  */
void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
    __HAL_UNLOCK(huart);

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
	{
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);      		//清除溢出中断
	    volatile uint32_t tmp;
	    tmp = huart->Instance->RDR; // 读出数据寄存器清空错误
	    (void)tmp;
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	}
}
//UART1中断接收函数
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t length)
{
    // Vofa_UART_Receive(buffer, length);
    if (Vofa_Receive_flag==0)
    {
        Vofa_UART_Receive(buffer, length);
        Vofa_Receive_flag=1;
    }
}
// //UART2中断接收函数
// static void UART2_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//     IMU_get_measer(buffer,length);
// }
// //UART3中断接收函数
// static void UART3_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//     if (VS_Receive_flag==0)
//     {
//         Visual_Get_Convert(buffer,length);
//         VS_Receive_flag=1;
//     }
// }
// //UART4中断接收函数
// static void UART4_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//     HAL_UART_Transmit(&huart4,buffer,length,0xff);
// }
// //UART5中断接收函数
// static void UART5_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//     IMU_get_measer(buffer,length);
// }
// //UART6中断接收函数
// static void UART6_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//
//     //HAL_UART_Transmit(&huart6,buffer,length,0xff);
// }
//UART7中断接收函数
static void UART7_Receive_Serve(uint8_t *buffer, uint8_t length)
{
    //DMA要开circle模式
    IMU_get_measer(buffer,length);
}
//UART8中断接收函数
// static void UART8_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//
//     //HAL_UART_Transmit(&huart8,buffer,length,0xff);
// }
//UART9中断接收函数
// static void UART9_Receive_Serve(uint8_t *buffer, uint8_t length)
// {
//
//     //HAL_UART_Transmit(&huart9,buffer,length,0xff);
// }
//UART10中断接收函数
static void UART10_Receive_Serve(uint8_t *buffer, uint8_t length)
{
    //DMA无需开circle模式
    if (VS_Receive_flag==0)
    {
        Visual_Get_Convert2(buffer,length);
    }
}
