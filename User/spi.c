#include "stm32f10x.h"
//#include <stm32f10x_conf.h>

///************************************************
//�������� �� SPI_GPIO_Configuration
//��    �� �� SPI��������
//��    �� �� ��
//�� �� ֵ �� ��
//��    �� �� strongerHuang
//*************************************************/
//void SPI_GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;

//  /* SPI1: SCK�������� */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  /* SPI1: MOSI���ù��� */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//}

///************************************************
//�������� �� SPI_Configuration
//��    �� �� SPI����
//��    �� �� ��
//�� �� ֵ �� ��
//��    �� �� strongerHuang
//*************************************************/
//void SPI_Configuration(void)
//{
//  SPI_InitTypeDef  SPI_InitStructure;

//  /* SPI ��ʼ������ */
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; /*!< 2-line uni-directional data mode enable *///SPI_Direction_1Line_Rx
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //����Ϊ��SPI
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI���ͽ��� 8 λ֡�ṹ
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         //ʱ�ӿ���Ϊ��
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       //���ݲ����ڵڶ���ʱ����
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //������� NSS �ź�
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //������Ԥ��ƵֵSPI_BaudRatePrescaler_4
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //���ݴ���� MSB λ��ʼ
//  SPI_InitStructure.SPI_CRCPolynomial = 7;                           //���������� CRCֵ����Ķ���ʽ
//  SPI_Init(SPI1, &SPI_InitStructure);

//  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);                   //ʹ�ܽ����ж�

//  SPI_Cmd(SPI1, ENABLE);                                             //ʹ��SPI1
//}

///************************************************
//�������� �� SPI_Initializes
//��    �� �� SPI��ʼ��
//��    �� �� ��
//�� �� ֵ �� ��
//��    �� �� strongerHuang
//*************************************************/
//void SPI_Initializes(void)
//{
//  SPI_GPIO_Configuration();
//  SPI_Configuration();
//}

/************************************************
�������� �� SPI_WriteReadByte
��    �� �� ��SPIд���ֽ�����
��    �� �� TxData --- ���͵��ֽ�����
�� �� ֵ �� ���������ֽ�����
��    �� �� strongerHuang
*************************************************/
uint8_t SPI_WriteReadByte(uint8_t TxData)
{
	//while(SPI_I2S_GetITStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetITStatus
  while((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI2->DR = TxData;

  while((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  return SPI2->DR;
}

