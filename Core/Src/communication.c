#include "communication.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "control.h"
#include "debug.h"
#include "utils.h"
#include "led.h"

uint8_t Communication_MessageChecksum(uint8_t *data)
{
    uint8_t checksum = 0;
    uint8_t length = data[3];

    for (int i = 0; i < length - 3 - 1; i++)
    {
        checksum ^= (data[3 + i] & 0xFF);
    }
    return checksum;
}

int Communication_GetRemoteSpeed(void) { return (NRF_RxBuf[7] << 8) | NRF_RxBuf[8]; }
int Communication_GetRemoteTurnning(void) { return (NRF_RxBuf[5] << 8) | NRF_RxBuf[6]; }

bool Communication_MessagePending(void)
{
    if (NRF24L01_GetRxLen() >= _MESSAGE_DATA_LENTH_)
    {
        if (NRF_RxBuf[_MESSAGE_DATA_LENTH_ - 1] == Communication_MessageChecksum(NRF_RxBuf))
        {
            LED_Toggle();
            // log_info("Message Received\r\n");
            return true;
        }
        else
        {
            NRF24L01_ClearRX();
            log_error("Message Checksum Error\r\n");
        }
    }
    return false;
}

void Communication_CheckMessage(void)
{
    static uint32_t tick = 0;

    if (Communication_MessagePending())
    {
        if (NRF_RxBuf[0] == '$' && NRF_RxBuf[1] == 'M')
        {
            int turnning = Communication_GetRemoteTurnning();
            int speed = Communication_GetRemoteSpeed();
            // log_info("S:%d\tT:%d\t", speed, turnning);

            speed = map(speed, 0, 4096, -_CONTROL_MAX_SPEED_, _CONTROL_MAX_SPEED_);
            turnning = map(4096 - turnning, 0, 4096, -_CONTROL_MAX_TURNNING_, _CONTROL_MAX_TURNNING_);
            // log_info("s:%d\tt:%d\t%d\t\r\n", speed, turnning, HAL_GetTick());

            Control_UpdateMoveSpeed(speed);
            Control_UpdateTurnningSpeed(turnning);

            NRF24L01_ClearRX();
            tick = HAL_GetTick();
        }
        else
        {
            log_error("Message Format Error\r\n");
            NRF24L01_ClearRX();
        }
    }
    else if (HAL_GetTick() - tick > _CONTROL_TIME_OUT_)
    {
        Control_EnterIdleState();
    }
}
