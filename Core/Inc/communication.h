#if !defined(_COMMUNICATION_H_)
#define _COMMUNICATION_H_

#define _SPEED_MAX_VAL_ 500
#define _TURNNING_MAX_VAL_ 300

// If the system does not receive any data within _CONTROL_TIME_OUT_ ms
// it whill enter idle state(standing still, do nothing)
#define _CONTROL_TIME_OUT_ 600

// Check data from remote controler and process instructions
void Communication_CheckMessage(void);

#endif // _COMMUNICATION_H_
