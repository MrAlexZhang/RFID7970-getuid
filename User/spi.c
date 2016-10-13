#include "stm32f10x.h"
//#include <stm32f10x_conf.h>

///************************************************
//函数名称 ： SPI_GPIO_Configuration
//功    能 ： SPI引脚配置
//参    数 ： 无
//返 回 值 ： 无
//作    者 ： strongerHuang
//*************************************************/
//void SPI_GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;

//  /* SPI1: SCK浮动输入 */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  /* SPI1: MOSI复用功能 */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//}

///************************************************
//函数名称 ： SPI_Configuration
//功    能 ： SPI配置
//参    数 ： 无
//返 回 值 ： 无
//作    者 ： strongerHuang
//*************************************************/
//void SPI_Configuration(void)
//{
//  SPI_InitTypeDef  SPI_InitStructure;

//  /* SPI 初始化定义 */
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; /*!< 2-line uni-directional data mode enable *///SPI_Direction_1Line_Rx
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //设置为主SPI
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI发送接收 8 位帧结构
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         //时钟空闲为低
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       //数据捕获于第二个时钟沿
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //软件控制 NSS 信号
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //波特率预分频值SPI_BaudRatePrescaler_4
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //数据传输从 MSB 位开始
//  SPI_InitStructure.SPI_CRCPolynomial = 7;                           //定义了用于 CRC值计算的多项式
//  SPI_Init(SPI1, &SPI_InitStructure);

//  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);                   //使能接收中断

//  SPI_Cmd(SPI1, ENABLE);                                             //使能SPI1
//}

///************************************************
//函数名称 ： SPI_Initializes
//功    能 ： SPI初始化
//参    数 ： 无
//返 回 值 ： 无
//作    者 ： strongerHuang
//*************************************************/
//void SPI_Initializes(void)
//{
//  SPI_GPIO_Configuration();
//  SPI_Configuration();
//}

/************************************************
函数名称 ： SPI_WriteReadByte
功    能 ： 对SPI写读字节数据
参    数 ： TxData --- 发送的字节数据
返 回 值 ： 读回来的字节数据
作    者 ： strongerHuang
*************************************************/
uint8_t SPI_WriteReadByte(uint8_t TxData)
{
	//while(SPI_I2S_GetITStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetITStatus
  while((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI2->DR = TxData;

  while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  return SPI2->DR;
}

