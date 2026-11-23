#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;


void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);


}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}

void USART1_RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能USART1 DMA接收
    huart1.Instance->CR3 |= USART_CR3_DMAR;
    
    // 清除IDLE标志
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
    // 禁用DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    // 等待DMA完全停止
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
    
    // 配置DMA外设地址
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    
    // 配置双缓冲区
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    
    // 设置数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    
    // 使能双缓冲模式
    hdma_usart1_rx.Instance->CR |= DMA_SxCR_DBM;
    
    // 使能循环模式
    hdma_usart1_rx.Instance->CR |= DMA_SxCR_CIRC;
    
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}





