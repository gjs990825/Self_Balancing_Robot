#include "nrf24l01.h"
#include <stdio.h>
#include <string.h>

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xAA, 0xBB, 0xCC, 0x00, 0x01}; //本地地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xAA, 0xBB, 0xCC, 0x00, 0x01}; //接收地址

uint8_t NRF_RxBuf[RX_PLOAD_WIDTH] = {0};
uint8_t NRF_rx_len = 0;

/* Memory Map */
#define NRF24L01_CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define NRF24L01_STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

/* Bit Mnemonics */
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
#define AW 0
#define ARD 4
#define ARC 0
#define PLL_LOCK 4
#define RF_DR 3
#define RF_PWR 6
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL 0
#define PLOS_CNT 4
#define ARC_CNT 0
#define TX_REUSE 6
#define FIFO_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0
#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define RF24_NOP 0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW 5
#define RF_DR_HIGH 3
#define RF_PWR_LOW 1
#define RF_PWR_HIGH 2

SPI_HandleTypeDef SPI1_Handle;

void NRF24L01_SPIInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // SPI1 SLCK-PA5 MOSI-PA7 MISO-PA6
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // CE-PB10 CSN-PB1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SPI1_Handle.Instance = SPI1;
    SPI1_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI1_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI1_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI1_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI1_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI1_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI1_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SPI1_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI1_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI1_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
    SPI1_Handle.Init.CRCPolynomial = 7;

    HAL_SPI_Init(&SPI1_Handle);
}

void NRF24L01_IRQCallBack(void)
{
    printf("NRF IRQ CallBack\r\n");

    uint8_t status = NRF24L01_SPIReadReg(R_REGISTER + NRF24L01_STATUS);

    if (status & (1 << RX_DR)) //接收中断
    {
        NRF_rx_len = NRF24L01_SPIReadReg(R_RX_PL_WID);
        if (NRF_rx_len < 33)
        {
            NRF24L01_SPIReadBuf(R_RX_PAYLOAD, NRF_RxBuf, NRF_rx_len); //接收
        }
        else
        {
            NRF24L01_SPIWriteReg(FLUSH_RX, 0xff); //清空缓冲区
        }
    }
    if (status & (1 << MAX_RT)) //达到最多次重发中断
    {
        if (status & (1 << 0)) //TX FIFO 溢出
        {
            NRF24L01_SPIWriteReg(FLUSH_TX, 0xff); //清空发送缓冲区
        }
    }
    NRF24L01_SPIWriteReg(W_REGISTER + NRF24L01_STATUS, status); //清除中断标志位
}

EXTI_HandleTypeDef EXTI_B0HandleStruct;

void NRF24L01_IRQInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_ConfigTypeDef EXTI_B0ConfigStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    // IRQ-PB0
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    EXTI_B0ConfigStruct.Line = EXTI_LINE_0;
    EXTI_B0ConfigStruct.Mode = EXTI_MODE_INTERRUPT;
    EXTI_B0ConfigStruct.Trigger = EXTI_TRIGGER_FALLING;
    EXTI_B0ConfigStruct.GPIOSel = EXTI_GPIOB;

    HAL_EXTI_RegisterCallback(&EXTI_B0HandleStruct, HAL_EXTI_COMMON_CB_ID, NRF24L01_IRQCallBack);
    HAL_EXTI_SetConfigLine(&EXTI_B0HandleStruct, &EXTI_B0ConfigStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{
    HAL_EXTI_IRQHandler(&EXTI_B0HandleStruct);
}

uint8_t NRF24L01_SPIRW(uint8_t data)
{
    uint8_t rx_data;

    HAL_SPI_TransmitReceive(&SPI1_Handle, &data, &rx_data, 1, 10);
    return rx_data;
}

void NRF24L01_PinCE(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, state);
}

void NRF24L01_PinCSN(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, state);
}

uint8_t NRF24L01_SPIWriteReg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF24L01_PinCSN(GPIO_PIN_RESET); /* 选通器件 */
    status = NRF24L01_SPIRW(reg);    /* 写寄存器地址 */
    NRF24L01_SPIRW(value);           /* 写数据 */
    NRF24L01_PinCSN(GPIO_PIN_SET);   /* 禁止该器件 */
    return status;
}

uint8_t NRF24L01_SPIReadReg(uint8_t reg)
{
    uint8_t reg_val;
    NRF24L01_PinCSN(GPIO_PIN_RESET); /* 选通器件 */
    NRF24L01_SPIRW(reg);             /* 写寄存器地址 */
    reg_val = NRF24L01_SPIRW(0);     /* 读取该寄存器返回数据 */
    NRF24L01_PinCSN(GPIO_PIN_SET);   /* 禁止该器件 */
    return reg_val;
}

uint8_t NRF24L01_SPIWriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    NRF24L01_PinCSN(GPIO_PIN_RESET); /* 选通器件 */
    status = NRF24L01_SPIRW(reg);    /* 写寄存器地址 */
    for (i = 0; i < uchars; i++)
    {
        NRF24L01_SPIRW(pBuf[i]); /* 写数据 */
    }
    NRF24L01_PinCSN(GPIO_PIN_SET); /* 禁止该器件 */
    return status;
}

uint8_t NRF24L01_SPIReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    NRF24L01_PinCSN(GPIO_PIN_RESET); /* 选通器件 */
    status = NRF24L01_SPIRW(reg);    /* 写寄存器地址 */
    for (i = 0; i < uchars; i++)
    {
        pBuf[i] = NRF24L01_SPIRW(0); /* 读取返回数据 */
    }
    NRF24L01_PinCSN(GPIO_PIN_SET); /* 禁止该器件 */
    return status;
}

void NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len)
{
    NRF24L01_PinCE(GPIO_PIN_RESET); //StandBy I模式

    NRF24L01_SPIWriteBuf(W_REGISTER + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
    NRF24L01_SPIWriteBuf(W_TX_PAYLOAD, tx_buf, len);                         // 装载数据
    NRF24L01_PinCE(GPIO_PIN_SET);                                            //置高CE，激发数据发送
}

void NRF24L01_TxPacketAP(uint8_t *tx_buf, uint8_t len)
{
    NRF24L01_PinCE(GPIO_PIN_RESET);          //StandBy I模式
    NRF24L01_SPIWriteBuf(0xa8, tx_buf, len); // 装载数据
    NRF24L01_PinCE(GPIO_PIN_SET);            //置高CE
}

const int kChannel = 80;
const int kMode = 3;

void NRF24L01_Init(void)
{
    NRF24L01_SPIInit();

    NRF24L01_PinCE(GPIO_PIN_RESET);
    NRF24L01_SPIWriteBuf(W_REGISTER + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址
    NRF24L01_SPIWriteBuf(W_REGISTER + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    //写TX节点地址
    NRF24L01_SPIWriteReg(W_REGISTER + EN_AA, 0x01);                          //使能通道0的自动应答
    NRF24L01_SPIWriteReg(W_REGISTER + EN_RXADDR, 0x01);                      //使能通道0的接收地址
    NRF24L01_SPIWriteReg(W_REGISTER + SETUP_RETR, 0x1a);                     //设置自动重发间隔时间:500us;最大自动重发次数:10次 2M波特率下
    NRF24L01_SPIWriteReg(W_REGISTER + RF_CH, kChannel);                      //设置RF通道为CHANAL
    NRF24L01_SPIWriteReg(W_REGISTER + RF_SETUP, 0x0f);                       //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    // NRF24L01_SPIWriteReg(W_REGISTER + RF_SETUP,0x07); 												//设置TX发射参数,0db增益,1Mbps,低噪声增益开启
    // NRF24L01_SPIWriteReg(W_REGISTER + RF_SETUP,0x27); 												//设置TX发射参数,0db增益,250Kbps,低噪声增益开启
    /////////////////////////////////////////////////////////
    if (kMode == 1) //RX
    {
        NRF24L01_SPIWriteReg(W_REGISTER + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度
        NRF24L01_SPIWriteReg(W_REGISTER + NRF24L01_CONFIG, 0x0f);    // IRQ收发完成中断开启,16位CRC,主接收
    }
    else if (kMode == 2) //TX
    {
        NRF24L01_SPIWriteReg(W_REGISTER + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度
        NRF24L01_SPIWriteReg(W_REGISTER + NRF24L01_CONFIG, 0x0e);    // IRQ收发完成中断开启,16位CRC,主发送
    }
    else if (kMode == 3) //RX2
    {
        NRF24L01_SPIWriteReg(FLUSH_TX, 0xff);
        NRF24L01_SPIWriteReg(FLUSH_RX, 0xff);
        NRF24L01_SPIWriteReg(W_REGISTER + NRF24L01_CONFIG, 0x0f); // IRQ收发完成中断开启,16位CRC,主接收

        NRF24L01_SPIRW(0x50);
        NRF24L01_SPIRW(0x73);
        NRF24L01_SPIWriteReg(W_REGISTER + 0x1c, 0x01);
        NRF24L01_SPIWriteReg(W_REGISTER + 0x1d, 0x06);
    }
    else //TX2
    {
        NRF24L01_SPIWriteReg(W_REGISTER + NRF24L01_CONFIG, 0x0e); // IRQ收发完成中断开启,16位CRC,主发送
        NRF24L01_SPIWriteReg(FLUSH_TX, 0xff);
        NRF24L01_SPIWriteReg(FLUSH_RX, 0xff);

        NRF24L01_SPIRW(0x50);
        NRF24L01_SPIRW(0x73);
        NRF24L01_SPIWriteReg(W_REGISTER + 0x1c, 0x01);
        NRF24L01_SPIWriteReg(W_REGISTER + 0x1d, 0x06);
    }
    NRF24L01_PinCE(GPIO_PIN_SET);

    NRF24L01_IRQInit();
}

bool NRF24L01_Check(void)
{
    uint8_t buf1[5];
    uint8_t i;
    /*写入5个字节的地址. */
    NRF24L01_SPIWriteBuf(W_REGISTER + TX_ADDR, TX_ADDRESS, 5);
    /*读出写入的地址 */
    NRF24L01_SPIReadBuf(TX_ADDR, buf1, 5);
    /*比较*/
    for (i = 0; i < 5; i++)
    {
        if (buf1[i] != TX_ADDRESS[i])
            break;
    }

    return (i == 5);
}

uint16_t NRF24L01_GetRxLen(void)
{
    return NRF_rx_len;
}

void NRF24L01_ClearRX(void)
{
    NRF_rx_len = 0;
    memset(NRF_RxBuf, 0, 16);
}
