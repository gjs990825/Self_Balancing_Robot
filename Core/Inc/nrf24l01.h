#if !defined(_NRF24L01_H_)
#define _NRF24L01_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define MODEL_RX 1  //普通接收
#define MODEL_TX 2  //普通发送
#define MODEL_RX2 3 //接收模式2,用于双向传输
#define MODEL_TX2 4 //发送模式2,用于双向传输

#define RX_PLOAD_WIDTH 255
#define TX_PLOAD_WIDTH 255
#define TX_ADR_WIDTH 5
#define RX_ADR_WIDTH 5

extern uint8_t NRF_RxBuf[RX_PLOAD_WIDTH];

void NRF24L01_Init(void);
uint8_t NRF24L01_SPIRW(uint8_t data);
uint8_t NRF24L01_SPIWriteReg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_SPIReadReg(uint8_t reg);
uint8_t NRF24L01_SPIWriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t NRF24L01_SPIReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
void NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len);
void NRF24L01_TxPacketAP(uint8_t *tx_buf, uint8_t len);

bool NRF24L01_Check(void);
uint16_t NRF24L01_GetRxLen(void);
void NRF24L01_ClearRX(void);

#endif // _NRF24L01_H_
