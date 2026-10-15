#ifndef __KEY_H
#define __KEY_H

#include "main.h"

extern volatile uint8_t key_states[4]; // Store the state of each key (pressed or not)

void Key_Init(void);
void Key_Scan(void);

#endif // __KEY_H
