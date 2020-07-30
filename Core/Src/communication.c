#include "communication.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "control.h"
#include "debug.h"
#include "utils.h"

int Communication_GetRemoteSpeed(void) { return (NRF_RxBuf[7] << 8) | NRF_RxBuf[8]; }
int Communication_GetRemoteTurnning(void) { return (NRF_RxBuf[5] << 8) | NRF_RxBuf[6]; }

void Communication_CheckMessage(void)
{
    static uint32_t tick = 0;

    if (NRF24L01_GetRxLen() != 0)
    {
        if (NRF_RxBuf[0] == '$' && NRF_RxBuf[1] == 'M')
        {
            int turnning = Communication_GetRemoteTurnning();
            int speed = Communication_GetRemoteSpeed();
            log_info("S:%d\tT:%d\t", speed, turnning);

            speed = map(speed, 0, 4096, -_SPEED_MAX_VAL_, _SPEED_MAX_VAL_);
            turnning = map(4096 - turnning, 0, 4096, -_TURNNING_MAX_VAL_, _TURNNING_MAX_VAL_);
            log_info("s:%d\tt:%d\t%d\t\r\n", speed, turnning, HAL_GetTick());

            Control_UpdateTargetSpeed(speed);
            Control_UpdateTurnningSpeed(turnning);

            NRF24L01_ClearRX();
            tick = HAL_GetTick();
        }
    }
    else if (HAL_GetTick() - tick > _CONTROL_TIME_OUT_)
    {
        Control_UpdateTargetSpeed(0);
        Control_UpdateTurnningSpeed(0);
    }
}
